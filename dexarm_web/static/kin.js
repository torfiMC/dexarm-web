// Same math as kinematics.py (ported from the DexArm firmware). mm and degrees.
const ARM = 150;
const OFFSET_ROT = 12.5;
const D = Math.PI / 180;
const MARGIN = 1.5;  // same safety margin as the server (REACH_MARGIN)
let Z_MIN = -105;    // user floor (server state z_min); the arm is never sent below it
export function setZMin(z) { if (typeof z === 'number') Z_MIN = z; }

export function fk(yaw, shoulder, elbow, moduleOffset) {
  const r = ARM * Math.cos(shoulder * D) + ARM * Math.cos(elbow * D) + moduleOffset + OFFSET_ROT;
  const z = ARM * Math.sin(shoulder * D) - ARM * Math.sin(elbow * D);
  return [r * Math.sin(yaw * D), r * Math.cos(yaw * D), z];
}

export function ik(x, y, z, moduleOffset) {
  const s = Math.hypot(x, y) - (moduleOffset + OFFSET_ROT);
  const d = Math.hypot(s, z);
  if (d > 2 * ARM || d < 1e-6) return null;
  const thb1 = Math.acos(d / (2 * ARM)) / D;
  const thb2 = Math.atan2(z, s) / D;
  return { yaw: Math.atan2(x, y) / D, shoulder: thb1 + thb2, elbow: thb1 - thb2 };
}

/** Where a jog/set on `axis` would end, starting from commanded pose `c` ({x,y,z,...}). */
export function predict(c, axis, value, relative, moduleOffset) {
  if (!c) return null;
  if (['x', 'y', 'z'].includes(axis)) {
    const p = { x: c.x, y: c.y, z: c.z };
    p[axis] = relative ? p[axis] + value : value;
    return [p.x, p.y, p.z];
  }
  const j = ik(c.x, c.y, c.z, moduleOffset);
  if (!j) return null;
  j[axis] = relative ? j[axis] + value : value;
  return fk(j.yaw, j.shoulder, j.elbow, moduleOffset);
}

// ---- reachability: mirrors kinematics.reach_error() and the server's jog planning ----

let LIMITS = null;
export async function loadLimits() {
  LIMITS = await (await fetch('/armlimit.json')).json();
}

export function reachable(x, y, z, moduleOffset) {
  if (!LIMITS) return true;
  if (z < Z_MIN - 1e-6) return false;
  if (z > LIMITS.zmax - MARGIN || z < LIMITS.zmin + MARGIN || y < MARGIN) return false;
  let idx = Math.trunc((Math.trunc(z + 0.5) + Math.abs(LIMITS.zmin)) / 2 + 0.5);
  idx = Math.max(0, Math.min(idx, LIMITS.table.length - 1));
  const [rmax, rmin] = LIMITS.table[idx];
  const s = Math.hypot(x, y) - (moduleOffset + OFFSET_ROT);
  return !(s < rmin + 1 + MARGIN || s > rmax - 1 - MARGIN);
}

/** Same checks the server makes before a jog: the straight path for X/Y/Z, ≤5° waypoints for joints. */
function jogOk(c, axis, value, moduleOffset) {
  const end = predict(c, axis, value, true, moduleOffset);
  if (!end) return false;
  if (['x', 'y', 'z'].includes(axis)) {
    const a = [c.x, c.y, c.z];
    const steps = Math.max(1, Math.ceil(Math.hypot(end[0] - a[0], end[1] - a[1], end[2] - a[2])));
    for (let i = 1; i <= steps; i++) {
      const t = i / steps;
      if (!reachable(a[0] + (end[0] - a[0]) * t, a[1] + (end[1] - a[1]) * t, a[2] + (end[2] - a[2]) * t, moduleOffset)) return false;
    }
    return true;
  }
  // joint jogs: ≤5° waypoints, and the straight chord between each pair is what actually gets sent
  const n = Math.max(1, Math.ceil(Math.abs(value) / 5));
  let prev = [c.x, c.y, c.z];
  for (let i = 1; i <= n; i++) {
    const p = predict(c, axis, (value * i) / n, true, moduleOffset);
    if (!p) return false;
    const len = Math.max(1, Math.ceil(Math.hypot(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2])));
    for (let k = 1; k <= len; k++) {
      const t = k / len;
      if (!reachable(prev[0] + (p[0] - prev[0]) * t, prev[1] + (p[1] - prev[1]) * t, prev[2] + (p[2] - prev[2]) * t, moduleOffset)) return false;
    }
    prev = p;
  }
  return true;
}

/** How far a jog can actually go: the full value, or the step that stops at the edge (0 = none). */
export function clampJog(c, axis, value, moduleOffset) {
  if (!c) return value;
  if (jogOk(c, axis, value, moduleOffset)) return value;
  let lo = 0, hi = Math.abs(value);
  const sign = Math.sign(value);
  for (let i = 0; i < 20; i++) {
    const mid = (lo + hi) / 2;
    if (jogOk(c, axis, sign * mid, moduleOffset)) lo = mid; else hi = mid;
  }
  lo = Math.floor(lo * 10) / 10;
  return lo < 0.1 ? 0 : sign * lo;
}

// ---- route planning (same rules as the server's _plan_route) ----

function straightOk(a, b, moduleOffset) {
  const steps = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / 1.0));  // 1 mm samples
  for (let i = 1; i <= steps; i++) {
    const t = i / steps;
    if (!reachable(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t, moduleOffset)) return false;
  }
  return true;
}

function pathOk(start, pts, moduleOffset) {
  let prev = start;
  for (const p of pts) { if (!straightOk(prev, p, moduleOffset)) return false; prev = p; }
  return true;
}

function arcPath(a, b, moduleOffset) {
  const cyl = p => [Math.atan2(p[0], p[1]) / D, Math.hypot(p[0], p[1]), p[2]];
  const [a0, r0, z0] = cyl(a), [a1, r1, z1] = cyl(b);
  const n = Math.max(1, Math.ceil(Math.max(Math.abs(a1 - a0) / 2, Math.abs(r1 - r0) / 2, Math.abs(z1 - z0) / 2)));
  const pts = [];
  for (let i = 1; i <= n; i++) {
    const t = i / n, ang = (a0 + (a1 - a0) * t) * D, r = r0 + (r1 - r0) * t;
    pts.push([r * Math.sin(ang), r * Math.cos(ang), z0 + (z1 - z0) * t]);
  }
  pts[pts.length - 1] = b.slice();
  return pathOk(a, pts, moduleOffset) ? pts : null;
}

function jointPath(a, b, moduleOffset) {
  const j0 = ik(...a, moduleOffset), j1 = ik(...b, moduleOffset);
  if (!j0 || !j1) return null;
  const keys = ['yaw', 'shoulder', 'elbow'];
  const n = Math.max(1, Math.ceil(Math.max(...keys.map(k => Math.abs(j1[k] - j0[k]))) / 5));
  const pts = [];
  for (let i = 1; i <= n; i++) {
    const t = i / n;
    pts.push(fk(...keys.map(k => j0[k] + (j1[k] - j0[k]) * t), moduleOffset));
  }
  pts[pts.length - 1] = b.slice();
  return pathOk(a, pts, moduleOffset) ? pts : null;
}

/** Route from a to b: {kind: 'straight'|'arc'|'joint', pts: [[x,y,z], ...]} or {error}. */
export function planRoute(a, b, moduleOffset, { curve = true, style = 'arc' } = {}) {
  if (!reachable(...b, moduleOffset)) return { error: 'target out of reach' };
  if (straightOk(a, b, moduleOffset)) return { kind: 'straight', pts: [b.slice()] };
  if (!curve) return { error: 'straight path is blocked (curving around is off)' };
  for (const kind of style === 'joint' ? ['joint', 'arc'] : ['arc', 'joint']) {
    const pts = kind === 'arc' ? arcPath(a, b, moduleOffset) : jointPath(a, b, moduleOffset);
    if (pts) return { kind, pts };
  }
  return { error: 'no route stays in reach' };
}
