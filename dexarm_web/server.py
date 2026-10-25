"""Web server: serves the UI and bridges a WebSocket to the arm controller."""
import argparse
import asyncio
import collections
import json
import os
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from .arm import Arm, list_serial_ports, fmt_xyz
from .bookmarks import Bookmarks

HERE = Path(__file__).parent
PROJECT = HERE.parent
# The arm's 3D model comes from https://github.com/Rotrics-Dev/URDF-of-DexArm (not bundled; install.sh downloads it).
URDF_CANDIDATES = [PROJECT / "vendor" / "URDF-of-DexArm" / "URDF", PROJECT.parents[1] / "URDF-of-DexArm" / "URDF"]


def urdf_dir():
    """Where DexArm.urdf and its STL meshes are: --urdf / $DEXARM_URDF_DIR, else vendor/, else a sibling checkout."""
    env = os.environ.get("DEXARM_URDF_DIR")
    for d in ([Path(env).expanduser()] if env else []) + URDF_CANDIDATES:
        if (d / "DexArm.urdf").is_file():
            return d
        if (d / "URDF" / "DexArm.urdf").is_file():   # pointed at the repo root instead of its URDF folder
            return d / "URDF"
    return None
NODE = PROJECT / "node_modules"
LOG_DIR = PROJECT / "logs"
SETTINGS_FILE = PROJECT / "settings.json"   # app settings that survive restarts (minimum Z, path planning)
bookmarks = Bookmarks(PROJECT / "bookmarks.yaml")


def load_settings():
    try:
        return json.loads(SETTINGS_FILE.read_text())
    except (OSError, ValueError):
        return {}


def save_settings(**changes):
    data = load_settings() | changes
    SETTINGS_FILE.write_text(json.dumps(data, indent=2) + "\n")
TZ = ZoneInfo("America/Los_Angeles")

clients: set[WebSocket] = set()
log_history = collections.deque(maxlen=500)
events: asyncio.Queue | None = None
arm: Arm | None = None
log_file = None


def open_log_file():
    """One log file per run, named after the start time in Pacific time."""
    global log_file
    LOG_DIR.mkdir(exist_ok=True)
    now = datetime.now(TZ)
    path = LOG_DIR / f"dexarm-web_{now:%Y-%m-%d_%H-%M-%S}_{now:%Z}.log"
    log_file = open(path, "a", buffering=1, encoding="utf-8")
    log_file.write(f"# dexarm-web started {now:%Y-%m-%d %H:%M:%S %Z}\n")
    print(f"Logging to {path}", flush=True)


def write_log(t, kind, text):
    if log_file:
        stamp = datetime.fromtimestamp(t, TZ).strftime("%H:%M:%S.%f")[:-3]
        log_file.write(f"{stamp} {kind:<5} {text}\n")


def emit_from_thread(loop):
    def emit(kind, payload):
        loop.call_soon_threadsafe(events.put_nowait, (kind, payload))
    return emit


async def broadcaster():
    while True:
        kind, payload = await events.get()
        if kind == "wire_dump":
            write_log(payload["lines"][0][0] if payload["lines"] else 0, "wire",
                      f"--- last {len(payload['lines'])} raw serial lines before: {payload['why']} ---")
            for t, d, line in payload["lines"]:
                write_log(t, "wire", f"{d} {line}")
            write_log(payload["lines"][-1][0] if payload["lines"] else 0, "wire", "--- end ---")
            continue
        if kind == "log":
            log_history.append(payload)
            write_log(payload["t"], payload["kind"], payload["text"])
        msg = json.dumps({"type": kind, "data": payload})
        for ws in list(clients):
            try:
                await ws.send_text(msg)
            except Exception:
                clients.discard(ws)


async def port_watcher():
    """Refresh the USB port list and auto-connect when a DexArm shows up."""
    last_attempt = 0.0
    loop = asyncio.get_running_loop()
    while True:
        ports = await asyncio.to_thread(list_serial_ports)
        arm.set_ports(ports)
        s = arm.snapshot()
        conn = s["conn"]
        if s["autoconnect"] and conn["status"] in ("disconnected", "error") and loop.time() - last_attempt > 5:
            dexarms = [p["device"] for p in ports if p["is_dexarm"]]
            if dexarms:
                last_attempt = loop.time()
                arm.control(("connect", dexarms[0]))
        if conn["status"] in ("connected", "halted") and conn["port"] not in [p["device"] for p in ports]:
            arm.control(("disconnect",))
        await asyncio.sleep(1.0)


@asynccontextmanager
async def lifespan(app):
    global events, arm
    events = asyncio.Queue()
    arm = Arm(emit_from_thread(asyncio.get_running_loop()))
    if "z_min" in (saved := load_settings()):
        arm.update(z_min=float(saved["z_min"]))
    if isinstance(saved.get("path"), dict):
        arm.set_path(curve=saved["path"].get("curve"), style=saved["path"].get("style"))
    tasks = [asyncio.create_task(broadcaster()), asyncio.create_task(port_watcher())]
    yield
    for t in tasks:
        t.cancel()


app = FastAPI(lifespan=lifespan)
app.mount("/static", StaticFiles(directory=HERE / "static"), name="static")
app.mount("/vendor/three", StaticFiles(directory=NODE / "three"), name="three")
app.mount("/vendor/urdf-loader", StaticFiles(directory=NODE / "urdf-loader" / "src"), name="urdf-loader")


@app.get("/urdf/STL/{name}")
async def urdf_mesh(name: str):
    d = urdf_dir()
    path = (d / "STL" / name).resolve() if d else None
    if not path or path.parent != (d / "STL").resolve() or not path.is_file():
        raise HTTPException(404, "mesh not found")
    return FileResponse(path)


@app.get("/")
async def index():
    return FileResponse(HERE / "static" / "index.html")


@app.get("/armlimit.json")
async def armlimit():
    """The firmware's reach table, so the browser can flag jog buttons that would go out of range."""
    return FileResponse(HERE / "armlimit.json")


@app.get("/urdf/dexarm.urdf")
async def urdf():
    """The arm without the optional gripper/suction modules (everything after the Modules comment)."""
    d = urdf_dir()
    if d is None:
        raise HTTPException(404, "DexArm.urdf not found: run ./install.sh or pass --urdf (see README)")
    text = (d / "DexArm.urdf").read_text()
    cut = text.find("<!-- Modules -->")
    if cut != -1:
        text = text[:cut] + "</robot>\n"
    return Response(text, media_type="application/xml")


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    await ws.send_text(json.dumps({"type": "hello", "data": {"state": arm.snapshot(), "log": list(log_history),
                                                            "queue": arm.queue_snapshot(),
                                                            "bookmarks": bookmarks.snapshot()}}))
    try:
        while True:
            msg = json.loads(await ws.receive_text())
            handle(msg)
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)


def handle_bookmark(t, msg):
    s = arm.snapshot()
    if t == "bm_add":
        if msg.get("source") == "arm":
            m = s.get("measured") or s.get("commanded")
            if not m:
                return arm._log("error", "No arm position yet; bookmark not added")
            xyz = [m["x"], m["y"], m["z"]]
        elif msg.get("source") == "cursor":
            xyz = s["cursor"]
        else:
            xyz = msg["xyz"]
        b = bookmarks.add(xyz, msg.get("name"))
        arm._log("info", f"Bookmarked {b['name']}: {fmt_xyz(b['xyz'])}")
    elif t == "bm_delete":
        bookmarks.delete(int(msg["id"]))
    elif t == "bm_rename":
        bookmarks.rename(int(msg["id"]), msg["name"])
    elif t == "bm_move":
        bookmarks.move(int(msg["id"]), [float(v) for v in msg["xyz"]])
    elif t == "bm_goto":
        b = bookmarks.get(int(msg["id"]))
        if b:
            arm.add_job({"kind": "goto", "xyz": b["xyz"]}, label=f"Go to bookmark \u201c{b['name']}\u201d")
        return
    events.put_nowait(("bookmarks", bookmarks.snapshot()))


def handle(msg):
    t = msg.get("type")
    if t == "estop":
        arm.estop()
    elif t == "kill":
        arm.set_killed(bool(msg["value"]))
    elif t.startswith("bm_"):
        handle_bookmark(t, msg)
    elif t == "path":
        save_settings(path=arm.set_path(curve=msg.get("curve"), style=msg.get("style")))
    elif t == "z_min":
        save_settings(z_min=arm.set_z_min(msg["value"]))
    elif t == "motion":
        arm.set_motion(accel=msg.get("accel"), jerk=msg.get("jerk"), save=bool(msg.get("save")))
    elif t == "motors":
        arm.set_motors(bool(msg["on"]))
    elif t == "connect":
        arm.update(autoconnect=True)
        arm.control(("connect", msg["port"]))
    elif t == "disconnect":
        arm.update(autoconnect=False)  # otherwise it would reconnect straight away
        arm.control(("disconnect",))
    elif t == "autoconnect":
        arm.update(autoconnect=bool(msg["value"]))
    elif t == "raw":
        for line in str(msg["cmd"]).splitlines():
            if line.strip():
                arm.add_job({"kind": "raw", "cmd": line.strip()})
    elif t in ("jog", "set"):
        arm.add_job({"kind": t, "axis": str(msg["axis"]), "value": float(msg["value"])})
    elif t == "goto":
        arm.add_job({"kind": "goto", "xyz": [float(v) for v in msg["xyz"]]})
    elif t == "feedrate":
        arm.update(feedrate=max(100.0, min(float(msg["value"]), 20000.0)))
    elif t == "live_feedrate":
        arm.update(live_feedrate=max(100.0, min(float(msg["value"]), 6000.0)))
    elif t == "queue_pause":
        arm.set_paused(bool(msg["value"]))
    elif t == "rerun":
        arm.rerun(int(msg["id"]))
    elif t == "cancel_job":
        arm.cancel_job(int(msg["id"]))
    elif t == "clear_history":
        arm.clear_history()
    elif t == "clear_all":
        arm.clear_all()
    elif t == "cursor":
        arm.set_cursor(msg["xyz"])
    elif t == "live":
        arm.set_live(bool(msg["value"]))


def main():
    ap = argparse.ArgumentParser(description="DexArm web control")
    ap.add_argument("--host", default="0.0.0.0", help="address to listen on (default: all interfaces)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--urdf", help="folder with DexArm.urdf and STL/ (default: vendor/URDF-of-DexArm/URDF)")
    args = ap.parse_args()
    if args.urdf:
        os.environ["DEXARM_URDF_DIR"] = args.urdf
    open_log_file()
    if urdf_dir() is None:
        print("3D model not found: run ./install.sh (or pass --urdf). The 3D view stays empty until then.",
              flush=True)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
