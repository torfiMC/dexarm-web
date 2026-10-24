// Time graph of sensed (encoder) positions and joint angles.
// Two stacked plots share the time axis: Position (mm) on top, Joints (°) below. Each has its own Y scale.
// Time only advances while recording, so idle periods take no space.
// By default a 10 s window scrolls right to left with time; zooming or panning switches to a manual view.

// Each plot's trio validated all-pairs on the dark surface (dataviz validator): position uses
// categorical slots 1-3 (blue, orange, aqua), joints use slots 4, 5, 7 (yellow, magenta, violet).
export const CHANNELS = [
  { key: 'x', label: 'X', plot: 0, color: '#3987e5' },
  { key: 'y', label: 'Y', plot: 0, color: '#d95926' },
  { key: 'z', label: 'Z', plot: 0, color: '#199e70' },
  { key: 'yaw', label: 'Base', plot: 1, color: '#c98500' },
  { key: 'shoulder', label: 'Shoulder', plot: 1, color: '#d55181' },
  { key: 'elbow', label: 'Elbow', plot: 1, color: '#9085e9' },
];
// Command-start markers: a pale tint, separated from every series by lightness rather than hue.
export const COMMAND_COLOR = '#e9e3b0';
const WINDOW_S = 10;                       // live view width
const PLOTS = [{ title: 'Position', unit: 'mm' }, { title: 'Joints', unit: '°' }];
const IDLE_S = 1.0;                        // recording pauses after this long without a recorded sample
const MAX_SAMPLES = 30000;
const MAX_STEP_S = 1.0;                    // cap on the time added between two recorded samples
const INK = { text: '#dfe4ea', muted: '#8a94a2', grid: '#1e242c', axis: '#2a313b', surface: '#0d1013' };
const M = { left: 52, right: 10, top: 6, bottom: 20, gap: 10 };

function niceStep(span, target) {
  const raw = span / Math.max(1, target);
  const mag = 10 ** Math.floor(Math.log10(raw));
  const n = raw / mag;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
}

function fmtTime(s) {
  if (s < 60) return `${s.toFixed(s < 10 ? 1 : 0)}s`;
  const m = Math.floor(s / 60);
  return `${m}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
}

export class Graph {
  constructor(canvas, { onStatus } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onStatus = onStatus;
    this.visible = Object.fromEntries(CHANNELS.map(c => [c.key, true]));
    this.trackWhenMotorsOff = false;
    this.movementOnly = true;   // record only when the tool point has moved at least minDelta
    this.minDelta = 1.0;        // mm, measured from the last recorded sample (so slow moves still add up)
    this.hold = false;          // PAUSE button: record nothing until REC
    this.clear();
    this.hover = null;
    this.pointerInside = false;
    this.dirty = true;

    new ResizeObserver(() => { this.dirty = true; }).observe(canvas.parentElement);
    this.wireInput();
    const loop = () => {
      if (this.dirty || (this.xMode === 'live' && this.recording)) this.draw();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  clear() {
    this.samples = [];        // {t, v: {...}, brk: first sample after a pause}
    this.clock = 0;           // recording time in seconds
    this.head = 0;            // right edge of the live window; only ever moves forward
    this.lastWall = null;
    this.recording = false;
    this.lastSeen = null;     // last reading, recorded or not
    this.lastRecorded = null;
    this.anchor = null;
    this.xMode = 'live';      // 'live' = scrolling 10 s window; 'all' = whole timeline; 'manual' = user zoomed/panned
    this.lastMotion = undefined;
    this.pendingCmd = false;  // a new command started; mark the next recorded sample
    this.x0 = 0; this.x1 = 10;
    this.yv = [{ auto: true, y0: 0, y1: 1 }, { auto: true, y0: 0, y1: 1 }];
    this.status = 'Waiting for data';
    this.onStatus?.(this.status, false);
    this.dirty = true;
  }

  setVisible(key, on) { this.visible[key] = on; this.dirty = true; }

  setHold(hold) {
    this.hold = hold;
    if (hold) { this.recording = false; this.setStatus('Paused', false); }
    else this.setStatus('Waiting for movement', false);
  }

  // ---------- data ----------

  /** Feed every state update; records only while the arm is moving (and motors are on, unless tracking hand moves). */
  feed(s) {
    const m = s.measured;
    const pushed = [];
    if (!m || CHANNELS.some(c => typeof m[c.key] !== 'number')) return pushed;
    const v = Object.fromEntries(CHANNELS.map(c => [c.key, m[c.key]]));
    const now = performance.now() / 1000;
    const prev = this.lastSeen;
    // compare with the last recorded point (or, before anything is recorded, the first reading)
    const ref = this.lastRecorded ?? (this.anchor ??= v);
    const dist = ref ? Math.hypot(v.x - ref.x, v.y - ref.y, v.z - ref.z) : 0;
    const moved = !!prev && (this.minDelta > 0 ? dist >= this.minDelta
      : CHANNELS.some(c => v[c.key] !== ref[c.key]));
    const motorsOff = s.motors === 'off';
    const allowed = !this.hold && (!motorsOff || this.trackWhenMotorsOff);
    if (s.motion_id !== this.lastMotion) {
      if (this.lastMotion !== undefined && s.motors !== 'off') this.pendingCmd = true;
      this.lastMotion = s.motion_id;
    }

    const settling = this.recording && allowed && this.movementOnly && now - (this.lastWall ?? now) < IDLE_S;
    if (allowed && prev && (moved || !this.movementOnly)) {
      if (!this.recording) {
        // resume: continue from the live window's edge (never behind it), from where the arm was standing
        this.clock = Math.max(this.clock, this.head);
        pushed.push(this.push(prev, true));
        this.lastWall = now;
      }
      this.clock += Math.min(MAX_STEP_S, Math.max(0.01, now - (this.lastWall ?? now)));
      pushed.push(this.push(v, false));
      this.lastWall = now;
      this.setStatus('Recording', true);
      this.recording = true;
    } else if (settling) {
      // below the threshold for a moment: keep the line going rather than marking a pause
    } else {
      if (this.recording || !prev) {
        this.setStatus(this.hold ? 'Paused' : !allowed ? 'Waiting: motors off' : prev ? 'Waiting: arm still' : 'Waiting for data', false);
      }
      this.recording = false;
    }
    this.lastSeen = v;
    return pushed;
  }

  /** Whether the graph is capturing this state at all (not paused, and motors on unless hand moves are tracked). */
  captures(s) { return !this.hold && (s.motors !== 'off' || this.trackWhenMotorsOff); }

  push(v, brk) {
    const sample = { t: this.clock, v: { ...v }, brk, cmd: this.pendingCmd };
    this.lastRecorded = sample.v;
    this.pendingCmd = false;
    this.samples.push(sample);
    if (this.samples.length > MAX_SAMPLES) this.samples.splice(0, this.samples.length - MAX_SAMPLES);
    const span = this.x1 - this.x0;
    if (this.xMode === 'manual' && this.x1 >= this.clock - span * 0.02 - 0.5) {
      // following the live edge: keep the newest data in view
      this.x1 = this.clock; this.x0 = this.clock - span;
    }
    this.dirty = true;
    return sample;
  }

  setStatus(text, rec) {
    if (text === this.status) return;
    this.status = text;
    this.onStatus?.(text, rec);
  }

  // ---------- view ----------

  layout() {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    const plotH = (h - M.top - M.bottom - M.gap) / 2;
    return {
      w, h,
      plots: [0, 1].map(i => ({ x: M.left, y: M.top + i * (plotH + M.gap), w: w - M.left - M.right, h: plotH })),
    };
  }

  /** Recording time right now. Between samples it keeps running with the wall clock, and it never goes
   *  backwards (a sample arriving early or recording pausing just holds it), so the window scrolls one way. */
  liveNow() {
    let t = this.clock;
    if (this.recording && this.lastWall != null)
      t += Math.min(MAX_STEP_S, Math.max(0, performance.now() / 1000 - this.lastWall));
    this.head = Math.max(this.head, t);
    return this.head;
  }

  xRange() {
    if (this.xMode === 'live') {
      const now = this.liveNow();
      return now < WINDOW_S ? [0, WINDOW_S] : [now - WINDOW_S, now];
    }
    if (this.xMode === 'all') {
      const last = this.samples.length ? this.samples[this.samples.length - 1].t : 0;
      return [0, Math.max(last, 1)];
    }
    return [this.x0, this.x1];
  }

  dataRange(plot, t0 = -Infinity, t1 = Infinity) {
    let lo = Infinity, hi = -Infinity;
    for (const s of this.samples) {
      if (s.t < t0 || s.t > t1) continue;
      for (const c of CHANNELS) {
        if (c.plot !== plot || !this.visible[c.key]) continue;
        lo = Math.min(lo, s.v[c.key]); hi = Math.max(hi, s.v[c.key]);
      }
    }
    if (!isFinite(lo)) return null;
    if (hi - lo < 1) { const mid = (hi + lo) / 2; lo = mid - 0.5; hi = mid + 0.5; }
    const pad = (hi - lo) * 0.06;
    return [lo - pad, hi + pad];
  }

  yRange(plot) {
    const yv = this.yv[plot];
    if (yv.auto) return this.dataRange(plot) ?? [0, 1];
    return [yv.y0, yv.y1];
  }

  /** F: fit vertically to everything recorded (and keep fitting as data comes in). */
  fitY() { this.yv.forEach(y => { y.auto = true; }); this.dirty = true; }

  /** Shift+F: show the whole timeline (and keep showing it as it grows). */
  fitX() { this.xMode = 'all'; this.dirty = true; }

  /** L: back to the scrolling 10 s window. */
  live() { this.xMode = 'live'; this.dirty = true; }

  /** A: fit everything, time and values. */
  fitAll() { this.fitX(); this.fitY(); }

  plotAt(py) {
    const L = this.layout();
    return L.plots.findIndex(p => py >= p.y - M.gap / 2 && py <= p.y + p.h + M.gap / 2);
  }

  zoom(px, py, factor, doX, doY) {
    const L = this.layout();
    const plotIdx = Math.max(0, this.plotAt(py));
    const P = L.plots[plotIdx];
    if (doX) {
      const [a, b] = this.xRange();
      const tAt = a + ((px - P.x) / P.w) * (b - a);
      this.xMode = 'manual';
      this.x0 = tAt - (tAt - a) * factor;
      this.x1 = tAt + (b - tAt) * factor;
    }
    if (doY) {
      const [a, b] = this.yRange(plotIdx);
      const vAt = b - ((py - P.y) / P.h) * (b - a);
      const yv = this.yv[plotIdx];
      yv.auto = false;
      yv.y0 = vAt - (vAt - a) * factor;
      yv.y1 = vAt + (b - vAt) * factor;
    }
    this.dirty = true;
  }

  pan(dx, dy, plotIdx) {
    const L = this.layout();
    const P = L.plots[plotIdx];
    if (dx) {
      const [a, b] = this.xRange();
      const dt = (-dx / P.w) * (b - a);
      this.xMode = 'manual';
      this.x0 = a + dt; this.x1 = b + dt;
    }
    if (dy) {
      const [a, b] = this.yRange(plotIdx);
      const dv = (dy / P.h) * (b - a);
      Object.assign(this.yv[plotIdx], { auto: false, y0: a + dv, y1: b + dv });
    }
    this.dirty = true;
  }

  // ---------- input ----------

  wireInput() {
    const c = this.canvas;
    c.addEventListener('wheel', e => {
      e.preventDefault();  // also stops Ctrl+wheel from zooming the page
      const r = c.getBoundingClientRect();
      const d = e.deltaY || e.deltaX;  // Shift+wheel arrives as horizontal scroll in some browsers
      const factor = Math.exp(Math.sign(d) * Math.min(Math.abs(d), 120) * 0.0018);
      const onlyX = e.ctrlKey || e.metaKey, onlyY = e.shiftKey;
      this.zoom(e.clientX - r.left, e.clientY - r.top, factor, !onlyY, !onlyX);
    }, { passive: false });

    let drag = null;
    c.addEventListener('pointerdown', e => {
      if (e.button !== 0) return;
      const r = c.getBoundingClientRect();
      drag = { x: e.clientX, y: e.clientY, plot: Math.max(0, this.plotAt(e.clientY - r.top)) };
      c.setPointerCapture(e.pointerId);
      c.focus();
    });
    c.addEventListener('pointermove', e => {
      const r = c.getBoundingClientRect();
      if (drag) {
        this.pan(e.clientX - drag.x, e.clientY - drag.y, drag.plot);
        drag.x = e.clientX; drag.y = e.clientY;
      }
      this.hover = { x: e.clientX - r.left, y: e.clientY - r.top };
      this.dirty = true;
    });
    c.addEventListener('pointerup', () => { drag = null; });
    c.addEventListener('pointerenter', () => { this.pointerInside = true; });
    c.addEventListener('pointerleave', () => { this.pointerInside = false; this.hover = null; this.dirty = true; });

    window.addEventListener('keydown', e => {
      const el = document.activeElement;
      const typing = el?.tagName === 'TEXTAREA' || el?.isContentEditable ||
        (el?.tagName === 'INPUT' && !['checkbox', 'radio', 'button', 'range'].includes(el.type));
      if (typing || !(this.pointerInside || document.activeElement === c)) return;
      if (e.key === 'f' || e.key === 'F') {
        e.preventDefault();
        if (e.shiftKey) this.fitX(); else this.fitY();
      } else if (e.key === 'l' || e.key === 'L') {
        e.preventDefault();
        this.live();
      } else if (e.key === 'a' || e.key === 'A') {
        e.preventDefault();
        this.fitAll();
      }
    });
  }

  // ---------- drawing ----------

  draw() {
    this.dirty = false;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const cw = this.canvas.clientWidth, chh = this.canvas.clientHeight;
    if (!cw || !chh) return;
    if (this.canvas.width !== Math.round(cw * dpr) || this.canvas.height !== Math.round(chh * dpr)) {
      this.canvas.width = Math.round(cw * dpr);
      this.canvas.height = Math.round(chh * dpr);
    }
    const g = this.ctx;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.fillStyle = INK.surface;
    g.fillRect(0, 0, cw, chh);
    g.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';

    const L = this.layout();
    const [t0, t1] = this.xRange();
    const X = (P, t) => P.x + ((t - t0) / (t1 - t0)) * P.w;

    L.plots.forEach((P, i) => {
      const [y0, y1] = this.yRange(i);
      const Y = v => P.y + P.h - ((v - y0) / (y1 - y0)) * P.h;

      // recessive grid + y labels
      g.strokeStyle = INK.grid; g.lineWidth = 1; g.fillStyle = INK.muted; g.textAlign = 'right'; g.textBaseline = 'middle';
      const ys = niceStep(y1 - y0, Math.max(2, Math.floor(P.h / 32)));
      for (let v = Math.ceil(y0 / ys) * ys; v <= y1; v += ys) {
        const py = Math.round(Y(v)) + 0.5;
        g.beginPath(); g.moveTo(P.x, py); g.lineTo(P.x + P.w, py); g.stroke();
        g.fillText(+v.toFixed(6) + '', P.x - 6, py);
      }
      // vertical time grid
      const ts = niceStep(t1 - t0, Math.max(2, Math.floor(P.w / 90)));
      for (let t = Math.ceil(t0 / ts) * ts; t <= t1; t += ts) {
        const px = Math.round(X(P, t)) + 0.5;
        g.beginPath(); g.moveTo(px, P.y); g.lineTo(px, P.y + P.h); g.stroke();
        if (i === 1) { g.textAlign = 'center'; g.textBaseline = 'top'; g.fillText(fmtTime(t), px, P.y + P.h + 4); }
      }
      g.strokeStyle = INK.axis;
      g.strokeRect(P.x + 0.5, P.y + 0.5, P.w - 1, P.h - 1);

      g.save();
      g.beginPath(); g.rect(P.x, P.y, P.w, P.h); g.clip();
      // vertical markers: command starts (pale, dashed) and resumes after idle (grey, dotted)
      g.lineWidth = 1;
      for (const s of this.samples) {
        if (!(s.cmd || s.brk) || s.t < t0 || s.t > t1) continue;
        const px = Math.round(X(P, s.t)) + 0.5;
        g.strokeStyle = s.cmd ? COMMAND_COLOR : INK.axis;
        g.globalAlpha = s.cmd ? 0.6 : 1;
        g.setLineDash(s.cmd ? [4, 3] : [1, 3]);
        g.beginPath(); g.moveTo(px, P.y); g.lineTo(px, P.y + P.h); g.stroke();
      }
      g.globalAlpha = 1;
      g.setLineDash([]);
      // series: 1px lines, broken at each resume, with a dot at every real sample
      g.lineWidth = 1; g.lineJoin = 'round'; g.lineCap = 'round';
      const pad = (t1 - t0) * 0.02;
      const inView = this.samples.filter(s => s.t >= t0 - pad && s.t <= t1 + pad);
      const spacing = inView.length > 1 ? P.w / ((inView.length - 1) || 1) : P.w;
      const dots = spacing >= 4;   // skip dots when zoomed out so far they'd merge into a bar
      for (const c of CHANNELS) {
        if (c.plot !== i || !this.visible[c.key]) continue;
        g.strokeStyle = c.color;
        g.beginPath();
        let started = false;
        for (const s of inView) {
          const px = X(P, s.t), py = Y(s.v[c.key]);
          if (!started || s.brk) { g.moveTo(px, py); started = true; } else g.lineTo(px, py);
        }
        g.stroke();
        if (dots) {
          g.fillStyle = c.color;
          g.beginPath();
          for (const s of inView) {
            const px = X(P, s.t), py = Y(s.v[c.key]);
            g.moveTo(px + 1.6, py); g.arc(px, py, 1.6, 0, Math.PI * 2);
          }
          g.fill();
        }
      }
      g.restore();

      // plot title in text ink
      g.fillStyle = INK.muted; g.textAlign = 'left'; g.textBaseline = 'top';
      g.fillText(`${PLOTS[i].title} (${PLOTS[i].unit})`, P.x + 6, P.y + 4);
    });

    if (!this.samples.length) {
      g.fillStyle = INK.muted; g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText('Nothing recorded yet. Graphing starts when the arm moves.', cw / 2, chh / 2);
    }
    this.drawHover(L, t0, t1, X);
  }

  drawHover(L, t0, t1, X) {
    const h = this.hover;
    if (!h || !this.samples.length) return;
    const P0 = L.plots[0];
    if (h.x < P0.x || h.x > P0.x + P0.w) return;
    const t = t0 + ((h.x - P0.x) / P0.w) * (t1 - t0);
    // nearest sample (samples are time-ordered)
    let lo = 0, hi = this.samples.length - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (this.samples[mid].t < t) lo = mid; else hi = mid; }
    const s = Math.abs(this.samples[lo].t - t) < Math.abs(this.samples[hi].t - t) ? this.samples[lo] : this.samples[hi];
    const g = this.ctx;
    const px = Math.round(X(P0, s.t)) + 0.5;
    g.strokeStyle = '#5b6573'; g.lineWidth = 1;
    for (const P of L.plots) { g.beginPath(); g.moveTo(px, P.y); g.lineTo(px, P.y + P.h); g.stroke(); }

    // dots on the lines (with a surface ring), then a tooltip in text ink with colour swatches
    L.plots.forEach((P, i) => {
      const [y0, y1] = this.yRange(i);
      for (const c of CHANNELS) {
        if (c.plot !== i || !this.visible[c.key]) continue;
        const py = P.y + P.h - ((s.v[c.key] - y0) / (y1 - y0)) * P.h;
        if (py < P.y || py > P.y + P.h) continue;
        g.beginPath(); g.arc(px, py, 4, 0, Math.PI * 2);
        g.fillStyle = c.color; g.fill(); g.lineWidth = 2; g.strokeStyle = INK.surface; g.stroke();
      }
    });
    const rows = [[null, `t ${fmtTime(s.t)}`]].concat(
      CHANNELS.filter(c => this.visible[c.key])
        .map(c => [c.color, `${c.label} ${s.v[c.key].toFixed(c.plot === 0 ? 1 : 2)}${c.plot === 0 ? ' mm' : '°'}`]));
    g.font = '11px ui-monospace, "SF Mono", Menlo, Consolas, monospace';
    const tw = Math.max(...rows.map(r => g.measureText(r[1]).width)) + 26;
    const th = rows.length * 15 + 8;
    let bx = px + 12, by = Math.max(4, h.y - th / 2);
    if (bx + tw > L.w - 4) bx = px - 12 - tw;
    by = Math.min(by, L.h - th - 4);
    g.fillStyle = 'rgba(24, 28, 34, 0.95)'; g.strokeStyle = INK.axis; g.lineWidth = 1;
    g.beginPath(); g.roundRect(bx, by, tw, th, 6); g.fill(); g.stroke();
    rows.forEach(([color, text], k) => {
      const ry = by + 6 + k * 15;
      if (color) { g.fillStyle = color; g.fillRect(bx + 8, ry + 3, 8, 8); }
      g.fillStyle = color ? INK.text : INK.muted; g.textAlign = 'left'; g.textBaseline = 'top';
      g.fillText(text, bx + (color ? 22 : 8), ry);
    });
  }
}
