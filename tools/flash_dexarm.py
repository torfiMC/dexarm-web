#!/usr/bin/env python3
# Back up and flash DexArm firmware over USB, the way Rotrics Studio does. See docs/dexarm-notes.md.
#
#   python tools/flash_dexarm.py enter            # app -> update bootloader (M2002 + M2003)
#   python tools/flash_dexarm.py info             # print hardware version (in bootloader)
#   python tools/flash_dexarm.py backup out.bin   # copy the installed firmware + settings off the arm
#   python tools/flash_dexarm.py flash image.bin  # write a firmware image, then start it
#
# Stop dexarm-web first (only one program can hold the port). Set DEXARM_PORT if the arm isn't /dev/ttyACM0.
# Use the image for YOUR hardware version (M2011), and take a backup before flashing.

import glob
import os
import re
import sys
import time
from pathlib import Path

import serial

PORT = os.environ.get("DEXARM_PORT", "/dev/ttyACM0")
SOH, EOT, ACK, NAK, CAN, C = 0x01, 0x04, 0x06, 0x15, 0x18, 0x43


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(index, payload):
    payload = payload.ljust(128, b"\x1a") if index else payload.ljust(128, b"\x00")
    c = crc16(payload)
    return bytes([SOH, index & 0xFF, (-index - 1) & 0xFF]) + payload + bytes([c >> 8, c & 0xFF])


def wait_port(timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        if glob.glob(PORT):
            try:
                return serial.Serial(PORT, 115200, timeout=0.2)
            except serial.SerialException:
                pass
        time.sleep(0.3)
    raise SystemExit(f"{PORT} did not come back")


def drain(ser, secs, show=True):
    out = b""
    end = time.time() + secs
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            out += chunk
            if show:
                sys.stdout.write(chunk.decode(errors="replace")); sys.stdout.flush()
    return out


def enter():
    ser = serial.Serial(PORT, 115200, timeout=0.2)
    drain(ser, 0.5, show=False)
    ser.write(b"M2002\r\n"); print(">> M2002")
    drain(ser, 1.5)
    ser.write(b"M2003\r\n"); print(">> M2003")
    try:
        drain(ser, 3)
    except serial.SerialException as e:
        print(f"\n[port dropped: {e}]")
    try:
        ser.close()
    except Exception:
        pass
    time.sleep(2)
    ser = wait_port()
    print("[reopened]")
    drain(ser, 4)
    ser.close()


def info():
    ser = serial.Serial(PORT, 115200, timeout=0.2)
    drain(ser, 1)
    ser.write(b"a5"); print(">> a5")
    drain(ser, 3)
    ser.close()


def wait_byte(ser, wanted, timeout):
    """Wait for one of `wanted` bytes; echo any text the bootloader prints meanwhile."""
    end = time.time() + timeout
    text = b""
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        v = b[0]
        if v in wanted and (v != C or not text[-1:].isalnum()):
            if text.strip():
                print(f"   [text] {text.decode(errors='replace').strip()}")
            return v
        text += b
    if text.strip():
        print(f"   [text] {text.decode(errors='replace').strip()}")
    return None


def flash(path):
    data = open(path, "rb").read()
    name = Path(path).name.replace("..", ".")
    frames = [frame(0, name.encode() + b"\x00" + f"{len(data)} ".encode())]
    frames += [frame(i + 1, data[i * 128:(i + 1) * 128]) for i in range((len(data) + 127) // 128)]
    print(f"{path}: {len(data)} bytes, {len(frames) - 1} data frames")

    ser = serial.Serial(PORT, 115200, timeout=0.2)
    drain(ser, 1)
    for attempt in range(3):
        ser.write(b"1"); print(">> 1  (download image to internal flash)")
        if wait_byte(ser, {C}, 8) == C:
            break
    else:
        raise SystemExit("bootloader never asked for the file (no 'C')")

    # header frame
    ser.write(frames[0])
    if wait_byte(ser, {ACK, NAK, CAN}, 10) != ACK:
        raise SystemExit("header frame not acknowledged")
    if wait_byte(ser, {C}, 10) != C:
        raise SystemExit("no 'C' after header")

    t0 = time.time()
    i = 1
    retries = 0
    while i < len(frames):
        ser.write(frames[i])
        r = wait_byte(ser, {ACK, NAK, CAN}, 10)
        if r == ACK:
            i += 1; retries = 0
            if i % 100 == 0 or i == len(frames):
                print(f"   {100 * (i - 1) // (len(frames) - 1)}%  frame {i - 1}/{len(frames) - 1}")
        elif r == CAN:
            raise SystemExit(f"bootloader cancelled at frame {i}")
        else:
            retries += 1
            print(f"   resend frame {i} ({'NAK' if r == NAK else 'timeout'})")
            if retries > 10:
                raise SystemExit(f"too many retries at frame {i}")
    print(f"data sent in {time.time() - t0:.1f}s")

    for _ in range(3):
        ser.write(bytes([EOT]))
        r = wait_byte(ser, {ACK, NAK}, 5)
        if r == ACK:
            break
    wait_byte(ser, {C}, 3)
    ser.write(frame(0, b""))   # empty header = end of batch
    wait_byte(ser, {ACK}, 5)

    out = drain(ser, 8)
    if b"Programming Completed Successfully" not in out:
        print("\n[did not see 'Programming Completed Successfully!' -- not sending run command]")
        ser.close()
        return
    time.sleep(1)
    ser.write(b"3"); print("\n>> 3  (run firmware)")
    try:
        drain(ser, 3)
    except serial.SerialException:
        pass
    ser.close()


STX = 0x02


def read_exact(ser, n, timeout=5):
    buf = b""
    end = time.time() + timeout
    while len(buf) < n and time.time() < end:
        buf += ser.read(n - len(buf))
    return buf


def backup(path):
    """Menu option 2: the bootloader sends the current app image over YMODEM."""
    ser = serial.Serial(PORT, 115200, timeout=0.2)
    drain(ser, 1, show=False)
    ser.write(b"2"); print(">> 2  (upload image from the internal flash)")
    time.sleep(0.5)
    pre = drain(ser, 1)
    data, size, expect, got_header = bytearray(), None, 0, False
    ser.write(bytes([C]))
    t0, last_c = time.time(), time.time()
    while True:
        b = ser.read(1)
        if not b:
            if not got_header and time.time() - last_c > 1:
                ser.write(bytes([C])); last_c = time.time()
            if time.time() - t0 > 120:
                raise SystemExit("backup timed out")
            continue
        v = b[0]
        if v in (SOH, STX):
            n = 128 if v == SOH else 1024
            rest = read_exact(ser, n + 4)
            if len(rest) != n + 4:
                ser.write(bytes([NAK])); continue
            idx, nidx, payload, crc = rest[0], rest[1], rest[2:2 + n], rest[2 + n:]
            if (idx + nidx) & 0xFF != 0xFF or crc16(payload) != (crc[0] << 8 | crc[1]):
                ser.write(bytes([NAK])); continue
            if not got_header:
                name, _, tail = payload.partition(b"\x00")
                m = re.match(rb"\d+", tail)
                size = int(m.group()) if m else None
                print(f"   header: name={name.decode(errors='replace')!r} size={size}")
                got_header, expect = True, 1
                ser.write(bytes([ACK, C]))
                continue
            if idx == (expect & 0xFF):
                data += payload; expect += 1
                if expect % 200 == 0:
                    print(f"   {len(data)} bytes")
            ser.write(bytes([ACK]))
        elif v == EOT:
            ser.write(bytes([ACK, C]))
            # final empty header
            h = ser.read(1)
            if h and h[0] in (SOH, STX):
                read_exact(ser, (128 if h[0] == SOH else 1024) + 4)
                ser.write(bytes([ACK]))
            break
        elif v == CAN:
            raise SystemExit("sender cancelled")
    if size:
        data = data[:size]
    open(path, "wb").write(data)
    print(f"saved {len(data)} bytes to {path}")
    drain(ser, 2)
    ser.close()


if __name__ == "__main__":
    if sys.argv[1] == "backup":
        backup(sys.argv[2]); sys.exit()
    cmd = sys.argv[1]
    if cmd == "enter":
        enter()
    elif cmd == "info":
        info()
    elif cmd == "flash":
        flash(sys.argv[2])
