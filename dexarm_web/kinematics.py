"""DexArm kinematics, ported from Marlin_For_DexArm (module/dexarm/dexarm.cpp).

World coordinates are the firmware's: X right, Y forward, Z up, in mm.

Joint angles are physical angles in degrees:
  yaw      base rotation, 0 = straight forward, + = clockwise seen from above (toward +X)
  shoulder upper arm elevation above horizontal
  elbow    forearm angle below horizontal

Firmware "Theta" values (M114) relate to these as:
  theta_a = yaw, theta_b = 105.24 - shoulder, theta_c = -9.60 - elbow
"""
import json
import math
from pathlib import Path

ARM = 150.0          # upper arm and forearm length, mm
OFFSET_ROT = 12.5    # firmware OFFSET_ROT; total horizontal offset = module offset + this
START_B = 105.24
START_C = 99.60

_limits = json.loads((Path(__file__).parent / "armlimit.json").read_text())


def fk(yaw, shoulder, elbow, module_offset):
    """Joint angles (deg) -> world position (mm)."""
    r = ARM * math.cos(math.radians(shoulder)) + ARM * math.cos(math.radians(elbow)) + module_offset + OFFSET_ROT
    z = ARM * math.sin(math.radians(shoulder)) - ARM * math.sin(math.radians(elbow))
    return r * math.sin(math.radians(yaw)), r * math.cos(math.radians(yaw)), z


def ik(x, y, z, module_offset):
    """World position (mm) -> joint angles (deg), or None if out of reach."""
    s = math.hypot(x, y) - (module_offset + OFFSET_ROT)
    d = math.hypot(s, z)
    if d > 2 * ARM or d < 1e-6:
        return None
    thb1 = math.degrees(math.acos(d / (2 * ARM)))
    thb2 = math.degrees(math.atan2(z, s))
    yaw = math.degrees(math.atan2(x, y))
    return yaw, thb1 + thb2, thb1 - thb2


def thetas_to_joints(a, b, c):
    return a, START_B - b, (90.0 - START_C) - c


def reach_error(x, y, z, module_offset, margin=0.0):
    """Same checks as dexarm_position_is_reachable(), optionally `margin` mm stricter.
    Returns a reason string, or None if reachable."""
    if z > _limits["zmax"] - margin or z < _limits["zmin"] + margin:
        return f"Z {z:.1f} is outside {_limits['zmin'] + margin:.0f}..{_limits['zmax'] - margin:.0f} mm"
    if y < margin:
        return f"Y {y:.1f} is below {margin:g} (the firmware can't turn the base past 90°)"
    idx = int((int(z + 0.5) + abs(_limits["zmin"])) / 2 + 0.5)
    idx = max(0, min(idx, len(_limits["table"]) - 1))
    rmax, rmin = _limits["table"][idx]
    s = math.hypot(x, y) - (module_offset + OFFSET_ROT)
    lo, hi = rmin + 1 + margin, rmax - 1 - margin
    if s < lo or s > hi:
        return f"horizontal reach {s:.1f} mm is outside {lo:.0f}..{hi:.0f} mm at Z {z:.0f}"
    return None
