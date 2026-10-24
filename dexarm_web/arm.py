"""Serial connection to the DexArm.

One worker thread owns the port. It runs the command queue one job at a time
(each move is waited out before the next job starts), polls position (M114) and
encoders (M895) in between, and streams live-track moves when that's switched on.

STOP (M410) bypasses everything: Marlin's emergency parser handles it at once,
even while the arm is busy.
"""
import collections
import math
import queue
import re
import threading
import time

import serial
from serial.tools import list_ports

from . import kinematics as kin

DEXARM_VID_PID = (0x0483, 0x5740)
POLL_INTERVAL = 0.25
DEFAULT_TIMEOUT = 8.0                       # seconds without any reply; "busy" messages extend it
LONG_COMMANDS = ("M1112", "G28", "M400", "M889", "G4")
MOTION_WORDS = ("G0", "G1", "G2", "G3")
MISMATCH_LIMIT_DEG = 2.0                    # refuse moves if firmware and encoders disagree by more
MISMATCH_LIMIT_MM = 6.0                     # same check in millimetres at the tool point
ERROR_WORDS = ("beyond limit", "error", "unknown command", "halted", "kill")
MAX_JOBS = 400                              # history kept in memory
HOME_XYZ = (0.0, 300.0, 0.0)
LIVE_MIN_STEP_MM = 0.5                      # ignore live-track changes smaller than this
REACH_MARGIN = 1.5                          # mm inside the firmware's limits, so it never truncates a move
Z_MIN_DEFAULT = -105.0                      # floor below which the arm is never sent (the table, for the attached module)
Z_MIN_RANGE = (-127.0, 100.0)
STEP_TIME = 0.1                             # moves are streamed in pieces of about this many seconds


def step_mm(feed):
    """Length of one streamed piece at this feedrate (mm/min)."""
    return min(5.0, max(0.5, feed / 60.0 * STEP_TIME))

XYZ_RE = re.compile(r"X:(-?[\d.]+)\s+Y:(-?[\d.]+)\s+Z:(-?[\d.]+)")
CAL_RE = re.compile(r"M889 X(-?\d+) Y(-?\d+) Z(-?\d+)")
ACCEL_RE = re.compile(r"M204 P([\d.]+)")
JERK_RE = re.compile(r"M205 .*?X([\d.]+)")
ROUTE_LABEL = {"arc": "curved around the base", "joint": "joint sweep"}
ACCEL_RANGE = (10.0, 600.0)   # mm/s^2; firmware allows up to 3000 (M201), shipped at 60
JERK_RANGE = (1.0, 20.0)      # mm/s; shipped at 10
OFFSET_RE = re.compile(r"FRONT_MODULE_OFFSET\s*(-?[\d.]+)")
FW_RE = re.compile(r"FIRMWARE_NAME:([^ ]+ [^ ]+)")
WORD_RE = re.compile(r"([XYZF])\s*(-?[\d.]+)", re.I)

JOINT_AXES = ("yaw", "shoulder", "elbow")
CART_AXES = ("x", "y", "z")
AXIS_LABEL = {"x": ("X", "mm"), "y": ("Y", "mm"), "z": ("Z", "mm"),
              "yaw": ("Base", "°"), "shoulder": ("Shoulder", "°"), "elbow": ("Elbow", "°")}


class Refused(Exception):
    """A job that can't or mustn't run (out of reach, position mismatch, ...)."""


class Aborted(Exception):
    """STOP or the kill switch interrupted a job."""


def list_serial_ports():
    out = []
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        if p.vid is None and not p.device.startswith(("/dev/ttyACM", "/dev/ttyUSB")):
            continue  # skip built-in legacy ttyS ports
        out.append({
            "device": p.device,
            "description": p.description if p.description != "n/a" else "",
            "vid_pid": f"{p.vid:04x}:{p.pid:04x}" if p.vid is not None else "",
            "serial": p.serial_number or "",
            "is_dexarm": (p.vid, p.pid) == DEXARM_VID_PID,
        })
    return out


def pose_from_xyz(x, y, z, module_offset):
    j = kin.ik(x, y, z, module_offset)
    pose = {"x": x, "y": y, "z": z, "yaw": None, "shoulder": None, "elbow": None}
    if j:
        pose.update(yaw=j[0], shoulder=j[1], elbow=j[2])
    return pose


def fmt_xyz(p):
    return f"X{p[0]:.1f} Y{p[1]:.1f} Z{p[2]:.1f}"


def job_label(spec):
    k = spec["kind"]
    if k == "raw":
        return spec["cmd"]
    if k in ("jog", "set"):
        name, unit = AXIS_LABEL.get(spec["axis"], (spec["axis"], ""))
        v = spec["value"]
        return f"{name} {v:+g}{unit}" if k == "jog" else f"{name} → {v:g}{unit}"
    if k == "goto":
        return f"Go to {fmt_xyz(spec['xyz'])}"
    return k


class Arm:
    def __init__(self, emit):
        self._emit = emit                   # emit(kind, payload); called from any thread
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._control = queue.Queue()       # connect / disconnect requests
        self._wake = threading.Event()      # nudges the worker (new job, play, cursor moved)
        self._abort = threading.Event()     # STOP / kill: cut the running job short
        self._need_release = False          # send M84 at the next chance (kill switch, motors switch)
        self._need_engage = False           # send M17 at the next chance (motors switch)
        self._immediate = collections.deque()  # settings commands that skip the queue (sent between moves)
        self._extra_oks = 0                 # "ok"s owed for commands written outside _send (M410)
        self._needs_sync = True             # re-sync firmware position from encoders (M17) before the next move
        self._io = {"cmds": 0, "bytes_out": 0, "bytes_in": 0, "rtt": [], "t0": time.time()}
        self._wire = collections.deque(maxlen=300)  # last raw serial lines, polls included (for crash reports)
        self._next_id = 1
        self._last_measured = None
        self._live_sent = None              # last live-track target actually sent
        self._live_err_logged = None
        self.ser = None
        self.jobs = []
        self.paused = False
        self.state = {
            "ports": [],
            "conn": {"status": "disconnected", "port": None, "error": None},
            "autoconnect": True,
            "firmware": None,
            "calibration": None,
            "module_offset": 109.5,
            "commanded": None,
            "measured": None,
            "moving": False,
            "motors": "unknown",
            "homed": False,
            "busy": False,
            "feedrate": 3000,
            "live_feedrate": 1200,
            "z_min": Z_MIN_DEFAULT,
            "path": {"curve": True, "style": "arc"},   # route around blocked straight paths (Move to, bookmarks, live)
            "motion": {"accel": None, "jerk": None, "unsaved": False},  # ease in/out (M204 / M205)
            "last_error": None,
            "killed": False,
            "target": None,            # [x, y, z] of the running or most recent move
            "target_active": False,    # a move toward target is in progress
            "target_path": None,       # planned route [[x, y, z], ...] from the move's start to target
            "cursor": [0.0, 250.0, 0.0],
            "cursor_error": None,      # why the cursor position is unreachable, if it is
            "live": False,
            "motion_id": 0,            # increments each time a command starts moving the arm
            "serial": None,            # {cmd_per_s, bytes_in_per_s, bytes_out_per_s, poll_ms}
        }
        threading.Thread(target=self._worker, daemon=True).start()

    # =============== public API (any thread) ===============

    def snapshot(self):
        with self._lock:
            return dict(self.state)

    def queue_snapshot(self):
        with self._lock:
            return {"jobs": [dict(j) for j in self.jobs], "paused": self.paused}

    def update(self, **kw):
        with self._lock:
            self.state.update(kw)
        self._emit("state", self.snapshot())

    def _queue_changed(self):
        self._emit("queue", self.queue_snapshot())

    def control(self, item):
        self._control.put(item)
        self._wake.set()

    # ----- queue -----

    def add_job(self, spec, label=None):
        if self.state["killed"]:
            self._log("error", f"Kill switch is on; ignored {label or job_label(spec)}")
            return
        if self.state["live"]:
            # live track holds the queue; a command the user just asked for takes over instead of waiting
            self.update(live=False)
            self._log("info", f"Live track off: {label or job_label(spec)} takes over")
        with self._lock:
            job = {"id": self._next_id, "spec": spec, "label": label or job_label(spec), "detail": "",
                   "status": "queued", "created": time.time(), "started": None, "finished": None,
                   "error": None, "target": None}
            self._next_id += 1
            self.jobs.append(job)
            self._trim()
        self._queue_changed()
        self._wake.set()

    def _record(self, spec, label, start, target):
        """Add an already-running entry to the queue (history only; the worker never picks it up)."""
        now = time.time()
        with self._lock:
            job = {"id": self._next_id, "spec": spec, "label": label,
                   "detail": f"{fmt_xyz(start)} → {fmt_xyz(target)}", "status": "running",
                   "created": now, "started": now, "finished": None, "error": None, "target": list(target)}
            self._next_id += 1
            self.jobs.append(job)
            self._trim()
        self._queue_changed()
        return job

    def rerun(self, job_id):
        with self._lock:
            job = next((j for j in self.jobs if j["id"] == job_id), None)
        if not job:
            return
        if job["target"] and job["spec"]["kind"] in ("jog", "set", "goto"):
            # a move is repeated as "go to where it ended", so it means the same thing from anywhere
            self.add_job({"kind": "goto", "xyz": list(job["target"])},
                         label=f"Go to {fmt_xyz(job['target'])}  (from #{job_id})")
        else:
            self.add_job(dict(job["spec"]), label=f"{job['label']}  (from #{job_id})")

    def cancel_job(self, job_id):
        with self._lock:
            for j in self.jobs:
                if j["id"] == job_id and j["status"] == "queued":
                    j.update(status="cancelled", finished=time.time())
        self._queue_changed()

    def clear_history(self):
        with self._lock:
            self.jobs = [j for j in self.jobs if j["status"] in ("queued", "running")]
        self._queue_changed()

    def clear_all(self):
        """Cancel everything waiting and drop all history. A running item finishes and then disappears too."""
        with self._lock:
            self.jobs = [j for j in self.jobs if j["status"] == "running"]
        self._queue_changed()

    def set_paused(self, paused):
        with self._lock:
            self.paused = bool(paused)
        self._queue_changed()
        self._wake.set()

    def _cancel_queued(self, why):
        n = 0
        with self._lock:
            for j in self.jobs:
                if j["status"] == "queued":
                    j.update(status="cancelled", finished=time.time(), error=why)
                    n += 1
        if n:
            self._queue_changed()
        return n

    def _trim(self):
        excess = len(self.jobs) - MAX_JOBS
        if excess > 0:
            done = [j for j in self.jobs if j["status"] not in ("queued", "running")][:excess]
            ids = {j["id"] for j in done}
            self.jobs = [j for j in self.jobs if j["id"] not in ids]

    # ----- cursor & live tracking -----

    def set_cursor(self, xyz):
        xyz = [float(v) for v in xyz]
        err = self._reach_error(xyz)
        self.update(cursor=xyz, cursor_error=err)
        self._wake.set()

    def set_live(self, on):
        if on:
            if self.state["killed"]:
                return self._log("error", "Kill switch is on; live track not started")
            if self.state["conn"]["status"] != "connected":
                return self._log("error", "Not connected; live track not started")
            self._live_sent = None  # the worker re-syncs from the encoders and checks before the first move
            self._live_err_logged = None
            self._log("info", "Live track on: the arm follows the cursor (queue is held)")
        elif self.state["live"]:
            self._log("info", "Live track off")
        self.update(live=bool(on))
        self._wake.set()

    # ----- stopping -----

    def estop(self):
        """Stop motion now, abort the running job and cancel everything queued."""
        self._abort.set()
        self._needs_sync = True
        if self.state["live"]:
            self.update(live=False)
        n = self._cancel_queued("cancelled by STOP")
        ser = self.ser
        if ser is None:
            self._log("error", "STOP: not connected")
            return
        try:
            with self._write_lock:
                ser.write(b"M410\r")
                self._wire.append((time.time(), ">", "M410"))
                # the emergency parser acts at once, but M410 still gets its own "ok" later
                self._extra_oks += 1
            self._log("tx", "M410  (emergency stop, sent immediately)" + (f"; cancelled {n} queued" if n else ""))
        except serial.SerialException as e:
            self._log("error", f"STOP failed: {e}")

    def _reach_error(self, p):
        """Firmware reach (with margin) plus the user's minimum Z. Returns a reason string, or None if OK."""
        z_min = self.state["z_min"]
        if p[2] < z_min - 1e-6:
            return f"Z {p[2]:.1f} is below the minimum Z ({z_min:g} mm)"
        return kin.reach_error(*p, self.state["module_offset"], REACH_MARGIN)

    def set_z_min(self, z):
        z = max(Z_MIN_RANGE[0], min(float(z), Z_MIN_RANGE[1]))
        self.update(z_min=z)
        self._log("info", f"Minimum Z set to {z:g} mm")
        self.set_cursor(self.state["cursor"])  # re-check the cursor against the new floor
        return z

    def set_motion(self, accel=None, jerk=None, save=False):
        """Ease in/out. Applies before the next move, without waiting in the queue (which live track holds)."""
        m = dict(self.state["motion"])
        if accel is not None:
            a = max(ACCEL_RANGE[0], min(float(accel), ACCEL_RANGE[1]))
            self._immediate.append(f"M204 P{a:g} T{a:g}")
            m.update(accel=a, unsaved=True)
        if jerk is not None:
            j = max(JERK_RANGE[0], min(float(jerk), JERK_RANGE[1]))
            self._immediate.append(f"M205 X{j:g} Y{j:g} Z{j:g}")
            m.update(jerk=j, unsaved=True)
        if save:
            self._immediate.append("M500")
            m.update(unsaved=False)
        self.update(motion=m)
        self._wake.set()

    def set_motors(self, on):
        """Motors switch. Acts right away instead of waiting in the queue (which live track holds)."""
        if self.state["killed"]:
            return self._log("error", "Kill switch is on; motors stay released")
        if on:
            self._need_release, self._need_engage = False, True
        else:
            if self.state["live"]:
                self.set_live(False)
            self._abort.set()  # cut a running move short
            with self._lock:
                pending = any(j["status"] == "queued" for j in self.jobs)
            if pending and not self.paused:
                self.set_paused(True)  # otherwise the next move would switch the motors straight back on
                self._log("info", "Queue paused because the motors were switched off")
            self._need_engage, self._need_release = False, True
        self._wake.set()

    def set_killed(self, on):
        """Kill switch. On: stop, flush the queue, release motors, refuse commands. Off: accept commands again."""
        if on:
            self.update(killed=True)
            self.estop()
            self._need_release = True
            self._log("error", "KILL SWITCH ON: motion stopped, queue flushed, motors being released")
        else:
            self._cancel_queued("cancelled by kill switch")
            self.update(killed=False, homed=False, last_error=None)
            self._log("info", "Kill switch off: arm accepts commands again (queue was flushed). "
                              "Home before moving if the arm was moved by hand.")
        self._wake.set()

    def set_ports(self, ports):
        with self._lock:
            changed = ports != self.state["ports"]
            self.state["ports"] = ports
        if changed:
            self._emit("state", self.snapshot())

    # =============== worker ===============

    def _worker(self):
        last_poll = 0.0
        while True:
            try:
                self._handle_control()
                connected = self.ser is not None and self.state["conn"]["status"] == "connected"
                if connected and self._need_release:
                    self._need_release = False
                    self._send("M84")
                    self._needs_sync = True
                    self.update(motors="off", homed=False)
                while connected and self._immediate:
                    self._send(self._immediate.popleft())
                if connected and self._need_engage:
                    self._need_engage = False
                    self._send("M17")      # also takes the position from the encoders
                    self._needs_sync = False
                    self.update(motors="on")
                if connected and not self.state["killed"]:
                    if self.state["live"]:
                        self._live_run()  # returns once the arm has reached the cursor
                    elif not self.paused and (job := self._next_job()):
                        self._run_job(job)
                        last_poll = time.time()
                        continue
                if connected and time.time() - last_poll >= POLL_INTERVAL:
                    self._poll()
                    last_poll = time.time()
                    self._report_io()
                self._wake.wait(0.05 if self.state["live"] else POLL_INTERVAL)
                self._wake.clear()
            except serial.SerialException as e:
                self._lost(f"Connection lost: {e}")
            except Exception as e:  # keep the worker alive no matter what
                self._log("error", f"Internal error: {e!r}")
                time.sleep(0.5)

    def _handle_control(self):
        while True:
            try:
                item = self._control.get_nowait()
            except queue.Empty:
                return
            if item[0] == "connect":
                self._connect(item[1])
            elif item[0] == "disconnect":
                self._close()
                self.update(conn={"status": "disconnected", "port": None, "error": None})
                self._log("info", "Disconnected")

    def _next_job(self):
        with self._lock:
            return next((j for j in self.jobs if j["status"] == "queued"), None)

    def _set_job(self, job, **kw):
        with self._lock:
            job.update(kw)
        self._queue_changed()

    def _run_job(self, job):
        self._abort.clear()
        self._set_job(job, status="running", started=time.time())
        self.update(busy=True, last_error=None)
        try:
            self._execute(job)
            if self._abort.is_set():
                raise Aborted()
            self._set_job(job, status="done", finished=time.time())
        except Aborted:
            self._set_job(job, status="aborted", finished=time.time(), error="stopped")
        except Refused as e:
            self._set_job(job, status="failed", finished=time.time(), error=str(e))
            self._log("error", f"#{job['id']} failed: {e}. Queue paused.")
            self.update(last_error=str(e))
            self.set_paused(True)
        except serial.SerialException:
            self._set_job(job, status="failed", finished=time.time(), error="connection lost")
            self.set_paused(True)
            raise
        finally:
            self.update(busy=False, target_active=False)

    def _execute(self, job):
        spec = job["spec"]
        kind = spec["kind"]
        if kind == "raw":
            return self._exec_raw(job, spec["cmd"].strip())
        off = self.state["module_offset"]
        start = self._fresh_start()
        if kind == "jog":
            value, waypoints = self._plan_clamped_jog(start, spec["axis"], spec["value"], off)
            if abs(value - spec["value"]) > 1e-6:
                name, unit = AXIS_LABEL.get(spec["axis"], (spec["axis"], ""))
                self._set_job(job, label=f"{job['label']}  (clamped to {value:+.1f}{unit}, edge of reach)")
        elif kind == "set":
            waypoints = self._plan_axis_move(start, spec["axis"], spec["value"], False, off)
        elif kind == "goto":
            waypoints, route = self._plan_route(start, tuple(spec["xyz"]), off)
            if route != "straight":
                self._set_job(job, label=f"{job['label']}  ({ROUTE_LABEL[route]})")
        else:
            raise Refused(f"unknown job kind {kind}")
        self._move(job, start, waypoints)

    # ----- planning -----

    def _mismatch(self):
        """Returns a reason string if firmware position and encoders disagree, else None."""
        c, m = self.state["commanded"], self.state["measured"]
        if c and m and c["yaw"] is not None and m["yaw"] is not None:
            worst = max(abs(c[k] - m[k]) for k in JOINT_AXES)
            if worst > MISMATCH_LIMIT_DEG:
                return (f"firmware and encoders disagree by {worst:.1f}°. "
                        "Home the arm (M1112) first")
        return None

    def _sync_from_encoders(self):
        """M17 enables the motors and makes the firmware take its position from the encoders, so the next
        move starts from where the arm really is (it may have been moved by hand, or stopped mid-move)."""
        if self._needs_sync:
            self._send("M17")
            self._needs_sync = False
            self.update(motors="on")

    def _fresh_start(self):
        self._sync_from_encoders()
        self._poll()
        c, m = self.state["commanded"], self.state["measured"]
        if not c:
            raise Refused("no position from firmware yet")
        if err := self._mismatch():
            raise Refused(err)
        if m and (gap := math.dist((c["x"], c["y"], c["z"]), (m["x"], m["y"], m["z"]))) > MISMATCH_LIMIT_MM:
            raise Refused(f"firmware position is {gap:.1f} mm from the encoder position. Home the arm (M1112) first")
        return (c["x"], c["y"], c["z"])

    def _plan_axis_move(self, start, axis, value, relative, off):
        x, y, z = start
        if axis in CART_AXES:
            cur = {"x": x, "y": y, "z": z}
            cur[axis] = cur[axis] + value if relative else value
            end = (cur["x"], cur["y"], cur["z"])
            self._check_straight(start, end, off)
            return [end]
        if axis in JOINT_AXES:
            j = kin.ik(x, y, z, off)
            if j is None:
                raise Refused("current position has no joint solution")
            j = dict(zip(JOINT_AXES, j))
            a0 = j[axis]
            a1 = a0 + value if relative else value
            n = max(1, math.ceil(abs(a1 - a0) / 5))  # ≤5° steps keep the G1 chords close to the arc
            pts = []
            for i in range(1, n + 1):
                j[axis] = a0 + (a1 - a0) * i / n
                pts.append(kin.fk(j["yaw"], j["shoulder"], j["elbow"], off))
            self._check_points(pts, off, start)
            return pts
        raise Refused(f"unknown axis {axis}")

    def _plan_clamped_jog(self, start, axis, value, off):
        """A jog that would leave the reachable area stops at its edge instead."""
        try:
            return value, self._plan_axis_move(start, axis, value, True, off)
        except Refused:
            pass
        lo, hi = 0.0, abs(value)
        sign = 1 if value > 0 else -1
        for _ in range(20):  # binary search for the largest step that still plans cleanly
            mid = (lo + hi) / 2
            try:
                self._plan_axis_move(start, axis, sign * mid, True, off)
                lo = mid
            except Refused:
                hi = mid
        lo = math.floor(lo * 10) / 10  # round down to 0.1 mm / 0.1°
        if lo < 0.1:
            raise Refused("already at the edge of the reachable area in that direction")
        return sign * lo, self._plan_axis_move(start, axis, sign * lo, True, off)

    def set_path(self, curve=None, style=None):
        p = dict(self.state["path"])
        if curve is not None:
            p["curve"] = bool(curve)
        if style in ("arc", "joint"):
            p["style"] = style
        self.update(path=p)
        return p

    def _plan_route(self, start, end, off):
        """Straight if that's legal; otherwise (if enabled) a curve around the base. Returns (waypoints, kind)."""
        if err := self._reach_error(end):
            raise Refused(f"target out of reach: {err}")
        try:
            self._check_straight(start, end, off)
            return [end], "straight"
        except Refused as e:
            blocked = e
        path = self.state["path"]
        if not path["curve"]:
            raise Refused(f"{blocked} (curving around is turned off)")
        order = ["arc", "joint"] if path["style"] == "arc" else ["joint", "arc"]
        for kind in order:
            try:
                return (self._plan_arc_path if kind == "arc" else self._plan_joint_path)(start, end, off), kind
            except Refused:
                continue
        raise Refused(f"{blocked}, and no curve around it stays in reach either")

    def _plan_arc_path(self, start, end, off):
        """Sweep around the base: interpolate base angle, distance from the base and height together."""
        def cyl(p):
            return math.degrees(math.atan2(p[0], p[1])), math.hypot(p[0], p[1]), p[2]
        (a0, r0, z0), (a1, r1, z1) = cyl(start), cyl(end)
        n = max(1, math.ceil(max(abs(a1 - a0) / 2, abs(r1 - r0) / 2, abs(z1 - z0) / 2)))  # ≤2° / ≤2 mm steps
        pts = []
        for i in range(1, n + 1):
            t = i / n
            a = math.radians(a0 + (a1 - a0) * t)
            r = r0 + (r1 - r0) * t
            pts.append((r * math.sin(a), r * math.cos(a), z0 + (z1 - z0) * t))
        pts[-1] = end
        self._check_points(pts, off, start)
        return pts

    def _plan_joint_path(self, start, end, off):
        """Path that interpolates joint angles, which stays inside the workspace better than a straight line."""
        if err := self._reach_error(end):
            raise Refused(f"target out of reach: {err}")
        j0, j1 = kin.ik(*start, off), kin.ik(*end, off)
        if j0 is None or j1 is None:
            raise Refused("no joint solution")
        n = max(1, math.ceil(max(abs(b - a) for a, b in zip(j0, j1)) / 5))
        pts = [kin.fk(*[a + (b - a) * i / n for a, b in zip(j0, j1)], off) for i in range(1, n + 1)]
        pts[-1] = end
        self._check_points(pts, off, start)
        return pts

    def _check_points(self, pts, off, start=None):
        """Check every straight piece between waypoints (the firmware checks each tiny segment it makes)."""
        prev = start
        for p in pts:
            if prev is not None:
                self._check_straight(prev, p, off)
            elif err := self._reach_error(p):
                raise Refused(f"out of reach: {err}")
            prev = p

    def _check_straight(self, a, b, off):
        steps = max(1, math.ceil(math.dist(a, b) / 1.0))  # 1 mm samples
        for i in range(1, steps + 1):
            t = i / steps
            p = tuple(a[k] + (b[k] - a[k]) * t for k in range(3))
            if err := self._reach_error(p):
                raise Refused(f"path leaves the reachable area ({err})")

    # ----- executing -----

    def _move(self, job, start, waypoints):
        target = list(waypoints[-1])
        self._set_job(job, target=target, detail=f"{fmt_xyz(start)} → {fmt_xyz(target)}")
        self._stream(start, waypoints, self.state["feedrate"])

    def _stream(self, start, waypoints, feed):
        """Send a path as short G1 pieces (see _pipeline)."""
        step = step_mm(feed)
        pieces, a = [], start
        for b in waypoints:
            n = max(1, math.ceil(math.dist(a, b) / step))
            pieces += [tuple(a[k] + (b[k] - a[k]) * i / n for k in range(3)) for i in range(1, n + 1)]
            a = b
        target = list(waypoints[-1])
        self.update(target=target, target_active=True, motors="on", motion_id=self.state["motion_id"] + 1,
                    target_path=[list(start)] + [list(w) for w in waypoints])
        self._log("tx", f"G1 F{feed:.0f} {fmt_xyz(start)} → {fmt_xyz(target)}  (streamed in {len(pieces)} pieces)")
        it = iter(pieces)
        self._pipeline(lambda: next(it, None), feed)
        self._finish_motion()

    def _pipeline(self, next_piece, feed):
        """Stream G1 pieces from next_piece() (None = done) while keeping the firmware's buffer full.

        The firmware buffers only ~80 ms of motion and caps speed to what it can stop within that buffer,
        so any gap between pieces makes the arm slow down and speed up again (a stutter). Here the next G1 is
        always already waiting in the firmware's queue while the current one runs, and position polls
        (M114 + M895) ride along behind it, so the 3D view still updates a few times a second.
        At most 4 commands are ever in the firmware's queue (its BUFSIZE).
        """
        ser = self.ser
        if ser is None:
            raise serial.SerialException("not connected")
        self._consume_extra_oks(ser)
        inflight = collections.deque()
        state = {"poll": False, "m114": None, "refused": None}

        def write(cmd):
            self._write_counted(ser, cmd)
            inflight.append(cmd)

        def pop_one():
            cmd = inflight.popleft()
            lines = self._read_reply(ser, cmd, quiet=True)
            if cmd == "M114":
                state["m114"] = lines
            elif cmd == "M895":
                state["poll"] = False
                self._apply_poll(state["m114"] or [], lines, can_resync=False)
            elif any("beyond limit" in l.lower() for l in lines):
                state["refused"] = "firmware refused part of the move (beyond limit)"

        try:
            while not self._abort.is_set() and not state["refused"]:
                p = next_piece()
                if p is None:
                    break
                write(f"G1 F{feed:.0f} X{p[0]:.2f} Y{p[1]:.2f} Z{p[2]:.2f}")
                if not state["poll"]:
                    write("M114")
                    write("M895")
                    state["poll"] = True
                # wait until only the G1 just written is outstanding; it keeps the firmware busy meanwhile
                while sum(c.startswith("G1") for c in inflight) > 1:
                    pop_one()
        finally:
            while inflight:
                pop_one()
        if state["refused"]:
            raise Refused(state["refused"])
        if self._abort.is_set():
            raise Aborted()

    def _finish_motion(self):
        if self._abort.is_set():
            raise Aborted()
        self._send("M400", quiet=True)
        self._poll()

    def _exec_raw(self, job, cmd):
        if not cmd:
            return
        word = cmd.split()[0].upper()
        if word in MOTION_WORDS:
            return self._exec_raw_move(job, cmd, word)
        if word == "M1112":
            self._set_job(job, target=list(HOME_XYZ))
            self.update(target=list(HOME_XYZ), target_active=True, target_path=None)

        lines = self._send(cmd)
        errors = [l for l in lines if any(w in l.lower() for w in ERROR_WORDS)]

        if word in ("M84", "M18"):
            self._needs_sync = True
            self.update(motors="off", homed=False)
        elif word in ("M17", "G28", "M1112", "M1111"):
            self.update(motors="on")
        if word == "M889":
            for line in lines:
                if m := re.search(r"SET X VALUE: (\d+)\s+Y VALUE: (\d+)\s+Z VALUE: (\d+)", line):
                    self.update(calibration=[int(v) for v in m.groups()])
        if errors:
            raise Refused(errors[0])
        if word in ("M1112", "M17"):
            self._needs_sync = False  # both set the firmware position from the encoders
        if word == "M1112":
            self.update(homed=True)

    def _exec_raw_move(self, job, cmd, word):
        """Raw G0-G3. Plain X/Y/Z/F moves are reach-checked and streamed; anything fancier is refused
        unless it can't leave the workspace (no X/Y/Z)."""
        self._sync_from_encoders()
        self._poll()
        c = self.state["commanded"]
        if not c:
            raise Refused("no position from firmware yet")
        start = (c["x"], c["y"], c["z"])
        words = {k.upper(): float(v) for k, v in WORD_RE.findall(cmd)}
        letters = {w[0].upper() for w in cmd.split()[1:] if w}
        if word in ("G2", "G3") or not letters <= set("XYZF"):
            raise Refused("only plain G0/G1 with X/Y/Z/F are accepted here (arcs and other words aren't reach-checked)")
        if "F" in words and not ({"X", "Y", "Z"} & set(words)):
            self._send(cmd)  # feedrate-only, no motion
            return
        tgt = (words.get("X", start[0]), words.get("Y", start[1]), words.get("Z", start[2]))
        self._check_straight(start, tgt, self.state["module_offset"])
        self._set_job(job, target=list(tgt), detail=f"→ {fmt_xyz(tgt)}")
        self._stream(start, [tgt], words.get("F", self.state["feedrate"]))

    def _live_run(self):
        """Follow the cursor in short pieces until the arm is there (or live track stops)."""
        off = self.state["module_offset"]
        if self._live_sent is None:
            self._sync_from_encoders()
            self._poll()
            c = self.state["commanded"]
            if not c or self._mismatch():
                self._log("error", "Live track stopped: firmware and encoder positions disagree. Home the arm.")
                self.update(live=False)
                return
            self._live_sent = (c["x"], c["y"], c["z"])
            self._log("info", f"Live track starting from {fmt_xyz(self._live_sent)}")
        feed = self.state["live_feedrate"]
        first = [True]
        entry = [None]   # queue entry for the live move in progress

        def close_entry(status, error=None):
            if entry[0]:
                self._set_job(entry[0], status=status, finished=time.time(), error=error)
                entry[0] = None

        route = {"target": None, "pts": []}   # planned path toward the current cursor (straight or curved)

        def next_piece():
            if not self.state["live"] or self.state["killed"]:
                return None
            cursor = tuple(self.state["cursor"])
            start = self._live_sent
            if math.dist(cursor, start) < LIVE_MIN_STEP_MM:
                return None
            if route["target"] != cursor:
                try:
                    pts, kind = self._plan_route(start, cursor, off)
                except Refused as e:
                    # hold position rather than send anything the firmware would cut short
                    self.update(cursor_error=str(e))
                    route["target"] = None
                    return None
                route.update(target=cursor, pts=list(pts), kind=kind)
                self.update(target_path=[list(start)] + [list(p) for p in pts])
            # advance one step along the planned path
            left, a, pts = step_mm(feed), start, route["pts"]
            while pts and math.dist(a, pts[0]) <= left:
                left -= math.dist(a, pts[0])
                a = pts.pop(0)
            if pts:
                d = math.dist(a, pts[0])
                a = tuple(a[k] + (pts[0][k] - a[k]) * left / d for k in range(3))
            nxt = a
            if entry[0] is None or entry[0]["target"] != list(cursor):
                # a new live-track move (the cursor was released somewhere new): log it in the queue
                close_entry("done", "redirected") if entry[0] else None
                label = f"Live → {fmt_xyz(cursor)}"
                if route.get("kind", "straight") != "straight":
                    label += f"  ({ROUTE_LABEL[route['kind']]})"
                entry[0] = self._record({"kind": "goto", "xyz": list(cursor)}, label, start, cursor)
            self._live_sent = nxt
            if first[0]:
                first[0] = False
                self.update(motion_id=self.state["motion_id"] + 1)
            if not self.state["target_active"] or self.state["target"] != list(cursor):
                self.update(target=list(cursor), target_active=True, motors="on")
            return nxt

        self._abort.clear()
        try:
            self._pipeline(next_piece, feed)
            if entry[0]:
                reached = math.dist(self._live_sent, entry[0]["target"]) < LIVE_MIN_STEP_MM
                close_entry("done" if reached else "aborted",
                            None if reached else self.state.get("cursor_error") or "live track stopped")
        except (Refused, Aborted) as e:
            self._live_sent = None
            close_entry("failed" if isinstance(e, Refused) else "aborted", str(e) or None)
            if isinstance(e, Refused):
                self._log("error", f"Live track: {e}")
        if self.state["target_active"]:
            self.update(target_active=False)

    # =============== connection ===============

    def _connect(self, port):
        self._close()
        self.update(conn={"status": "connecting", "port": port, "error": None})
        self._log("info", f"Connecting to {port}")
        try:
            ser = serial.Serial(port, 115200, timeout=0.1, write_timeout=2)
        except serial.SerialException as e:
            msg = str(e)
            if "Permission denied" in msg:
                msg = "Permission denied. Add your user to the 'dialout' group (then log in again)."
            self.update(conn={"status": "error", "port": port, "error": msg})
            self._log("error", msg)
            return
        self.ser = ser
        time.sleep(0.3)
        ser.reset_input_buffer()
        info = self._send("M115", quiet=True)
        fw = next((FW_RE.search(l).group(1) for l in info if FW_RE.search(l)), None)
        if fw is None:
            self._close()
            self.update(conn={"status": "error", "port": port, "error": "No reply from firmware"})
            self._log("error", "Port opened but the arm didn't answer M115. Power-cycle the arm?")
            return
        # M115 only names the Marlin base; M2010 gives the DexArm release ("Firmware V2.3.5")
        dex = next((l.split()[-1] for l in self._send("M2010", quiet=True) if l.startswith("Firmware V")), None)
        if dex:
            fw = f"DexArm {dex} ({fw})"
        cal, offset = None, self.state["module_offset"]
        motion = {"accel": None, "jerk": None, "unsaved": False}
        for line in self._send("M503", quiet=True):
            if m := ACCEL_RE.search(line):
                motion["accel"] = float(m.group(1))
            if m := JERK_RE.search(line):
                motion["jerk"] = float(m.group(1))
            if m := CAL_RE.search(line):
                cal = [int(v) for v in m.groups()]
            if m := OFFSET_RE.search(line):
                offset = float(m.group(1))
        self.update(conn={"status": "connected", "port": port, "error": None}, firmware=fw,
                    calibration=cal, module_offset=offset, homed=False, motors="unknown", motion=motion,
                    last_error=None)
        self._log("info", f"Connected: {fw}, module offset {offset} mm, calibration {cal}")
        self._needs_sync = True
        if self.state["killed"]:
            self._need_release = True  # keep the motors released after a reconnect
        self._poll()
        if self.state["measured"] and self.state["cursor"] == [0.0, 250.0, 0.0]:
            m = self.state["measured"]
            self.set_cursor([round(m["x"], 1), round(m["y"], 1), round(m["z"], 1)])

    def _close(self):
        ser, self.ser = self.ser, None
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        self.update(commanded=None, measured=None, moving=False, busy=False, live=False, target_active=False)

    def _lost(self, msg):
        port = self.state["conn"]["port"]
        self._close()
        self.update(conn={"status": "disconnected", "port": None, "error": msg})
        self._log("error", f"{msg} ({port})")
        self._dump_wire(msg)
        if self._next_job():
            self.set_paused(True)

    # =============== talking to the firmware ===============

    def _send(self, cmd, quiet=False):
        """Send one command and collect reply lines until "ok". Returns the lines (without "ok")."""
        ser = self.ser
        if ser is None:
            raise serial.SerialException("not connected")
        self._consume_extra_oks(ser)
        if not quiet:
            self._log("tx", cmd)
        t0 = time.time()
        self._write_counted(ser, cmd)
        lines = self._read_reply(ser, cmd, quiet)
        if cmd in ("M114", "M895"):
            self._io["rtt"].append(time.time() - t0)
        return lines

    def _write_counted(self, ser, cmd):
        data = (cmd + "\r").encode()
        with self._write_lock:
            ser.write(data)
        self._wire.append((time.time(), ">", cmd))
        self._io["cmds"] += 1
        self._io["bytes_out"] += len(data)

    def _report_io(self):
        """Every couple of seconds, publish how busy the serial link is."""
        io, now = self._io, time.time()
        dt = now - io["t0"]
        if dt < 2.0:
            return
        rtt = io["rtt"]
        self.update(serial={
            "cmd_per_s": round(io["cmds"] / dt, 1),
            "bytes_in_per_s": round(io["bytes_in"] / dt),
            "bytes_out_per_s": round(io["bytes_out"] / dt),
            "poll_ms": round(1000 * sum(rtt) / len(rtt), 1) if rtt else None,
        })
        self._io = {"cmds": 0, "bytes_out": 0, "bytes_in": 0, "rtt": [], "t0": now}

    def _read_reply(self, ser, cmd, quiet):
        """Read reply lines for `cmd` up to its "ok"."""
        limit = 120.0 if cmd.upper().startswith(LONG_COMMANDS) else DEFAULT_TIMEOUT
        deadline = time.time() + limit
        lines = []
        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            self._io["bytes_in"] += len(raw)
            line = raw.decode(errors="replace").strip()
            if line:
                self._wire.append((time.time(), "<", line))
            if not line or line == "wait":
                continue
            if line.startswith("echo:busy"):
                deadline = time.time() + limit
                continue
            if line.startswith("ok"):
                return lines
            lines.append(line)
            low = line.lower()
            is_err = any(w in low for w in ERROR_WORDS)
            if is_err:
                self.update(last_error=line)
                if "halted" in low or "kill" in low:
                    self.update(conn={**self.state["conn"], "status": "halted",
                                      "error": "Arm halted itself. Power-cycle it."})
            if is_err or not quiet:
                self._log("error" if is_err else "rx", line)
        self._log("error", f"No 'ok' for {cmd} within {limit:.0f}s")
        self._dump_wire(f"no reply to {cmd}")
        self._extra_oks += 1  # if it turns up later, don't let it be taken as the next command's reply
        return lines

    def _consume_extra_oks(self, ser):
        """Read the "ok" replies for commands written directly (M410), so replies stay matched to commands."""
        deadline = time.time() + 1.0
        while self._extra_oks > 0 and time.time() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                self._wire.append((time.time(), "<", line))
            if line.startswith("ok"):
                self._extra_oks -= 1
            elif line and line != "wait" and not line.startswith("echo:busy"):
                self._log("rx", line)
        self._extra_oks = 0

    def _resync(self, why):
        """Replies got out of step with commands: drop whatever is pending and start clean."""
        self._log("error", f"Serial replies out of step ({why}); resynchronising")
        time.sleep(0.3)
        if self.ser:
            self.ser.reset_input_buffer()
        self._extra_oks = 0

    def _poll(self):
        self._apply_poll(self._send("M114", quiet=True), self._send("M895", quiet=True))

    def _apply_poll(self, cmd_lines, enc_lines, can_resync=True):
        # M114's reply has a "Count A:" line, M895's never does. Anything else means a reply went astray.
        if not any("Count" in l for l in cmd_lines) or any("Count" in l for l in enc_lines):
            if can_resync:  # (not mid-stream: flushing would drop replies to commands still in flight)
                self._resync("position replies didn't match M114/M895")
            return
        off = self.state["module_offset"]
        cmd = next((XYZ_RE.search(l) for l in cmd_lines if XYZ_RE.search(l)), None)
        enc = next((XYZ_RE.search(l) for l in enc_lines if XYZ_RE.search(l)), None)
        commanded = pose_from_xyz(*map(float, cmd.groups()), off) if cmd else None
        measured = pose_from_xyz(*map(float, enc.groups()), off) if enc else None
        moving = False
        if measured and self._last_measured:
            moving = any(abs(measured[k] - self._last_measured[k]) > 0.3 for k in CART_AXES)
        self._last_measured = measured
        self.update(commanded=commanded, measured=measured, moving=moving)

    def _dump_wire(self, why):
        """Hand the recent raw serial traffic to the log file (not shown in the browser)."""
        self._emit("wire_dump", {"why": why, "lines": list(self._wire)})

    def _log(self, kind, text):
        self._emit("log", {"t": time.time(), "kind": kind, "text": text})
