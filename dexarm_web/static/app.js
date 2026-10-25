import { Viewer } from '/static/viewer.js';
import { predict, clampJog, loadLimits, reachable, setZMin, planRoute } from '/static/kin.js';
import { Graph, CHANNELS, COMMAND_COLOR } from '/static/graph.js';

const $ = sel => document.querySelector(sel);
const STEPS = { cart: [50, 20, 10, 1], joint: [20, 10, 1] };
const AXES = {
  cart: [
    { key: 'x', name: 'X', sub: '+ right', unit: 'mm' },
    { key: 'y', name: 'Y', sub: '+ forward', unit: 'mm' },
    { key: 'z', name: 'Z', sub: '+ up', unit: 'mm' },
  ],
  joint: [
    { key: 'yaw', name: 'Base', sub: '+ clockwise from above', unit: '°' },
    { key: 'shoulder', name: 'Shoulder', sub: 'upper arm above horizontal', unit: '°' },
    { key: 'elbow', name: 'Elbow', sub: 'forearm below horizontal', unit: '°' },
  ],
};
const STATUS_GLYPH = { queued: '○', running: '▶', done: '✓', failed: '✕', aborted: '■', cancelled: '–' };

let ws = null;
let state = null;
let queueState = { jobs: [], paused: false };
let viewer = null;
let graph = null;
const debugPose = new URLSearchParams(location.search).get('pose');  // e.g. ?pose=30,53.6,53.6

// ---------- websocket ----------

function connectWs() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => setWsStatus(true);
  ws.onclose = () => { setWsStatus(false); setTimeout(connectWs, 1000); };
  ws.onmessage = ev => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'hello') {
      $('#log').textContent = '';
      msg.data.log.forEach(addLog);
      renderQueue(msg.data.queue);
      render(msg.data.state);
      renderBookmarks(msg.data.bookmarks || []);
    } else if (msg.type === 'state') {
      render(msg.data);
    } else if (msg.type === 'queue') {
      renderQueue(msg.data);
    } else if (msg.type === 'log') {
      addLog(msg.data);
    } else if (msg.type === 'bookmarks') {
      renderBookmarks(msg.data);
    }
  };
}

function send(msg) {
  if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

function setWsStatus(up) {
  const el = $('#ws-status');
  el.textContent = up ? 'server online' : 'server offline';
  el.className = 'pill ' + (up ? 'ok' : 'bad');
  if (!up) setControlsEnabled(false, 'Not connected to the server.');
}

// ---------- rendering ----------

const fmt = (v, d = 1) => (typeof v === 'number' ? v.toFixed(d) : '—');

function render(s) {
  state = s;
  setZMin(s.z_min);  // before the jog rows are drawn, so red buttons and clamping use the current floor
  if (s.path) {
    $('#path-curve').checked = !!s.path.curve;
    if (document.activeElement !== $('#path-style')) $('#path-style').value = s.path.style;
    $('#path-style').disabled = !s.path.curve;
  }
  if (s.z_min !== lastZMin) {
    lastZMin = s.z_min;
    if (viewer) viewer.guideKey = null;   // reach guides depend on the floor
    if (bookmarks.length) renderBookmarks(bookmarks);
  }
  renderBmButtons();
  const conn = s.conn;
  const statusText = {
    connected: 'Connected', connecting: 'Connecting…', disconnected: 'Disconnected',
    error: 'Connection error', halted: 'Arm halted',
  }[conn.status] || conn.status;
  $('#conn-text').textContent = statusText;
  $('#conn-dot').className = 'dot ' + ({ connected: 'ok', connecting: 'warn', error: 'bad', halted: 'bad' }[conn.status] || '');
  $('#conn-port').textContent = conn.port || '';
  const err = $('#conn-error');
  err.hidden = !conn.error;
  err.textContent = conn.error || '';
  $('#btn-disconnect').disabled = !['connected', 'connecting', 'halted'].includes(conn.status);
  $('#autoconnect').checked = s.autoconnect;

  renderPorts(s);
  renderArmState(s);
  renderRows(s);
  renderCursor(s);
  renderKill(s);

  const zm = $('#z-min');
  if (document.activeElement !== zm && s.z_min != null) zm.value = s.z_min;
  const sp = $('#speed');
  if (!sp.matches(':active')) { sp.value = s.feedrate; $('#speed-val').textContent = Math.round(s.feedrate); }
  const mo = s.motion || {};
  const online = s.conn?.status === 'connected';
  for (const [k, id, fmt] of [['accel', 'ease-accel', v => v.toFixed(0)], ['jerk', 'ease-jerk', v => v.toFixed(0)]]) {
    const el = $('#' + id);
    if (mo[k] != null && !el.matches(':active')) { el.value = mo[k]; $('#' + id + '-val').textContent = fmt(mo[k]); }
    el.disabled = !online || mo[k] == null;
  }
  $('#ease-save').disabled = !online || !mo.unsaved;
  $('#ease-save').classList.toggle('unsaved', !!mo.unsaved);

  const connected = conn.status === 'connected';
  const killed = !!s.killed;
  $('#btn-home').disabled = !connected || killed;
  const mt = $('#motors-toggle');
  mt.disabled = !connected || killed;
  mt.setAttribute('aria-checked', s.motors === 'on' ? 'true' : s.motors === 'off' ? 'false' : 'mixed');
  mt.closest('.motor-switch').classList.toggle('is-on', s.motors === 'on');
  mt.closest('.motor-switch').classList.toggle('is-off', s.motors === 'off');
  setControlsEnabled(connected && !killed,
    killed ? 'Kill switch is on. Press it again to re-arm.' : connected ? null : 'Connect to the arm to move it.');
  renderQueueNote();

  const recorded = graph?.feed(s) || [];
  feedTrail(s, recorded);

  if (viewer && !debugPose) {
    viewer.setPoses(s.measured, s.commanded, s.moving);
    viewer.setTool(s.measured ? [s.measured.x, s.measured.y, s.measured.z] : null);
    viewer.setTarget(s.target, s.target_active, s.target_path);
    viewer.setCursor(s.cursor, !!s.cursor_error);
  }
}

// 3D trail: records whenever the tool point moves (motors on or off), skipping readings with no change.
let lastTrail = null;

// While the graph is capturing, the trail uses exactly the samples the graph recorded, so both show the same
// points. When the graph isn't (PAUSE, or motors off without hand tracking) the trail applies the same minimum
// delta itself, so hand moves still leave a trail.
function feedTrail(s, recorded) {
  const m = s.measured;
  if (!viewer || !m || !$('#trail-visible').checked) return;
  const group = s.motors === 'off' ? 'manual' : s.motion_id;
  const add = p => {
    if (lastTrail && p[0] === lastTrail[0] && p[1] === lastTrail[1] && p[2] === lastTrail[2]) return;
    viewer.addTrail(p, group, !lastTrail);
    lastTrail = p;
  };
  if (graph?.captures(s)) {
    for (const r of recorded) add([r.v.x, r.v.y, r.v.z]);
    return;
  }
  const p = [m.x, m.y, m.z];
  const step = Math.max(graph?.minDelta ?? 1, 1e-6);
  if (lastTrail && Math.hypot(p[0] - lastTrail[0], p[1] - lastTrail[1], p[2] - lastTrail[2]) < step) return;
  add(p);
}

function renderKill(s) {
  const killed = !!s.killed;
  const kill = $('#btn-kill');
  kill.classList.toggle('engaged', killed);
  kill.setAttribute('aria-pressed', String(killed));
  kill.querySelector('.kill-title').textContent = killed ? 'KILLED' : 'KILL';
  kill.querySelector('.kill-sub').textContent = killed ? 'press to re-arm (queue stays empty)' : 'stop · flush queue · motors off';
  $('#kill-banner').hidden = !killed;
}

function renderPorts(s) {
  const ul = $('#ports');
  ul.textContent = '';
  if (!s.ports.length) {
    ul.innerHTML = '<li class="empty">No USB serial ports found. Plug the arm in.</li>';
    return;
  }
  for (const p of s.ports) {
    const li = document.createElement('li');
    const active = s.conn.port === p.device && s.conn.status === 'connected';
    li.className = active ? 'active' : '';
    const dev = document.createElement('span');
    dev.className = 'dev';
    dev.textContent = p.device;
    if (p.is_dexarm) dev.insertAdjacentHTML('beforeend', '<span class="badge">DexArm</span>');
    const desc = document.createElement('span');
    desc.className = 'desc';
    desc.textContent = [p.description, p.vid_pid].filter(Boolean).join(' · ');
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.textContent = active ? 'Connected' : 'Connect';
    btn.disabled = active;
    btn.onclick = () => send({ type: 'connect', port: p.device });
    li.append(dev, btn, desc);
    ul.append(li);
  }
}

function renderArmState(s) {
  const c = s.commanded, m = s.measured;
  let tracking = '—', trackCls = '';
  if (c && m && typeof c.yaw === 'number' && typeof m.yaw === 'number') {
    const worst = Math.max(...AXES.joint.map(a => Math.abs(c[a.key] - m[a.key])));
    tracking = `${worst.toFixed(2)}° max`;
    trackCls = worst > 2 ? 'bad' : worst > 1 ? 'warn' : 'ok';
  }
  const rows = [
    ['Firmware', s.firmware || '—'],
    ['Homed', s.homed ? 'yes' : 'not this session', s.homed ? 'ok' : 'warn'],
    ['Motors', s.motors, s.motors === 'off' ? 'warn' : ''],
    ['Motion', s.moving ? 'moving' : 'still'],
    ['Serial', s.serial ? `${s.serial.cmd_per_s} cmd/s · ${((s.serial.bytes_in_per_s + s.serial.bytes_out_per_s) / 1024).toFixed(1)} kB/s` : '—'],
    ['Poll round trip', s.serial?.poll_ms != null ? `${s.serial.poll_ms} ms` : '—'],
    ['Encoder vs cmd', tracking, trackCls],
    ['Module offset', s.module_offset != null ? `${s.module_offset} mm` : '—'],
    ['Calibration', s.calibration ? s.calibration.join(' / ') : '—'],
    ['Last error', s.last_error || 'none', s.last_error ? 'bad' : ''],
  ];
  const dl = $('#armstate');
  dl.textContent = '';
  for (const [k, v, cls] of rows) {
    const dt = document.createElement('dt');
    dt.textContent = k;
    const dd = document.createElement('dd');
    dd.textContent = v;
    if (cls) dd.className = cls;
    dl.append(dt, dd);
  }
}

// ---------- jog rows ----------

const isJoint = k => AXES.joint.some(a => a.key === k);

function buildRows() {
  for (const [group, axes] of Object.entries(AXES)) {
    const host = $(group === 'cart' ? '#rows-cart' : '#rows-joint');
    for (const a of axes) {
      const row = document.createElement('div');
      row.className = 'row';
      row.dataset.axis = a.key;
      row.innerHTML = `
        <div class="row-head">
          <span><span class="name">${a.name}</span><span class="sub">${a.sub}</span></span>
          <span class="meas">enc <span class="enc">—</span> <span class="delta"></span></span>
        </div>
        <div class="row-ctl"></div>`;
      const ctl = row.querySelector('.row-ctl');
      ctl.classList.add(group);
      for (const s of STEPS[group]) ctl.append(stepButton(a.key, -s));
      const input = document.createElement('input');
      input.className = 'val';
      input.inputMode = 'decimal';
      input.title = `Commanded ${a.name} (${a.unit}). Type a value and press Enter.`;
      input.addEventListener('keydown', ev => {
        if (ev.key === 'Enter') {
          const v = parseFloat(input.value);
          if (Number.isFinite(v)) send({ type: 'set', axis: a.key, value: v });
          input.blur();
        } else if (ev.key === 'Escape') {
          input.blur();
        }
      });
      input.addEventListener('input', () => {
        const v = parseFloat(input.value);
        preview(Number.isFinite(v) ? [a.key, v, false] : null);
      });
      input.addEventListener('blur', () => preview(null));
      ctl.append(input);
      for (const s of [...STEPS[group]].reverse()) ctl.append(stepButton(a.key, s));
      host.append(row);
    }
  }
}

function stepButton(axis, delta) {
  const b = document.createElement('button');
  b.type = 'button';
  b.className = 'jog ' + (delta < 0 ? 'minus' : 'plus');
  b.textContent = (delta < 0 ? '−' : '+') + Math.abs(delta);
  b.dataset.axis = axis;
  b.dataset.delta = delta;
  b.onclick = () => send({ type: 'jog', axis, value: delta });
  b.addEventListener('mouseenter', () => preview([axis, delta, true]));
  b.addEventListener('mouseleave', () => preview(null));
  return b;
}

const moduleOffset = () => state?.module_offset ?? 109.5;

/** Show where a jog/set would end, based on the current commanded position. Jogs stop at the edge of reach. */
function preview(p) {
  if (!viewer) return;
  if (!p || !state?.commanded) return viewer.setPreview(null);
  let [axis, value, relative] = p;
  if (relative) value = clampJog(state.commanded, axis, value, moduleOffset());
  viewer.setPreview(value === 0 && relative ? null : predict(state.commanded, axis, value, relative, moduleOffset()));
}

/** Red buttons: the full step would leave the reachable area (clicking still moves, but only to the edge). */
function markJogLimits(s) {
  for (const b of document.querySelectorAll('.row-ctl button.jog')) {
    const delta = Number(b.dataset.delta);
    const unit = isJoint(b.dataset.axis) ? '°' : ' mm';
    const can = s.commanded ? clampJog(s.commanded, b.dataset.axis, delta, moduleOffset()) : delta;
    const out = can !== delta;
    b.classList.toggle('out', out);
    b.title = !out ? '' : can === 0 ? 'Already at the edge of reach in this direction'
      : `Out of reach. Clicking moves ${can > 0 ? '+' : ''}${can.toFixed(1)}${unit}, to the edge`;
  }
}

function renderRows(s) {
  markJogLimits(s);
  for (const row of document.querySelectorAll('.row')) {
    const k = row.dataset.axis;
    const c = s.commanded?.[k], m = s.measured?.[k];
    const input = row.querySelector('input.val');
    if (document.activeElement !== input) input.value = fmt(c, isJoint(k) ? 2 : 1);
    row.querySelector('.enc').textContent = fmt(m, isJoint(k) ? 2 : 1);
    const delta = row.querySelector('.delta');
    if (typeof c === 'number' && typeof m === 'number') {
      const d = c - m;
      const [warn, bad] = isJoint(k) ? [1, 2] : [3, 6];
      delta.textContent = `Δ ${d >= 0 ? '+' : ''}${d.toFixed(isJoint(k) ? 2 : 1)}`;
      delta.className = 'delta ' + (Math.abs(d) > bad ? 'bad' : Math.abs(d) > warn ? 'warn' : '');
    } else {
      delta.textContent = '';
    }
  }
}

function setControlsEnabled(on, why) {
  document.querySelectorAll('.row-ctl button, .row-ctl input, #btn-move-to')
    .forEach(el => { el.disabled = !on; });
  const hint = $('#jog-hint');
  hint.textContent = why || 'Hover a button or type a value to preview it in 3D. Δ = commanded − encoder.';
  hint.className = 'hint' + (why ? ' warn' : '');
}

// ---------- 3D cursor ----------

let lastCursor = null;

function showCursorCoords(xyz, error) {
  ['x', 'y', 'z'].forEach((k, i) => { $(`#cur-${k}`).textContent = xyz ? xyz[i].toFixed(1) : '—'; });
  $('.cursor-coords').classList.toggle('bad', !!error);
  const e = $('#cursor-error');
  e.hidden = !error;
  e.textContent = error ? `Out of reach: ${error}` : '';
}

function renderCursor(s) {
  if (!viewer?.dragging) {  // while dragging, onCursorDrag owns the coordinates and colour
    lastCursor = s.cursor;
    showCursorCoords(lastCursor, s.cursor_error);
  }
}

// While dragging, only the page updates (coordinates, colour, route preview). When the drag ends the arm moves
// there, unless the spot is out of reach or no route gets there (then only the cursor moves).
function onCursorDrag(xyz, final) {
  lastCursor = xyz;
  const ok = reachable(...xyz, moduleOffset());
  viewer?.setCursorReach(ok);
  let msg = ok ? null : "outside the arm's reach";
  if (final) {
    viewer?.setRoutePreview(null);
    const c = state?.commanded;
    const r = c && ok ? planRoute([c.x, c.y, c.z], xyz, moduleOffset(), state.path || {}) : null;
    if (r?.pts && !state?.killed) send({ type: 'cursor_go', xyz });
    else {
      send({ type: 'cursor', xyz });
      if (ok && r?.error) msg = `not moving: ${r.error}`;
    }
  } else {
    // while dragging, preview the route the arm would take from where it is holding (same planner as the server)
    const c = state?.commanded;
    if (c) {
      const start = [c.x, c.y, c.z];
      const r = planRoute(start, xyz, moduleOffset(), state.path || {});
      if (r.pts) viewer?.setRoutePreview([start, ...r.pts], true);
      else { viewer?.setRoutePreview([start, xyz], false); if (ok) msg = `no route: ${r.error}`; }
    }
  }
  showCursorCoords(xyz, msg);
}

$('#btn-move-to').onclick = () => { if (lastCursor) send({ type: 'goto', xyz: lastCursor }); };
// Uses the commanded position (where the arm is holding), not the encoder reading, which trails by the
// gear backlash; otherwise the next move would start with a small "catch-up". Never moves the arm.
$('#btn-cursor-to-arm').onclick = () => {
  const c = state?.commanded;
  if (c) send({ type: 'cursor', xyz: [c.x, c.y, c.z].map(v => +v.toFixed(1)) });
};

// Right-click on the cursor: same actions as the panel buttons. A right-drag still pans the view.
const menu = $('#cursor-menu');
let rightDown = null;

function hideMenu() { menu.hidden = true; }

function showMenu(clientX, clientY) {
  const vp = $('#viewport').getBoundingClientRect();
  const items = { move: $('#btn-move-to'), 'to-arm': $('#btn-cursor-to-arm') };
  for (const b of menu.querySelectorAll('button')) b.disabled = items[b.dataset.act].disabled;
  menu.hidden = false;
  const x = Math.min(clientX - vp.left, vp.width - menu.offsetWidth - 4);
  const y = Math.min(clientY - vp.top, vp.height - menu.offsetHeight - 4);
  menu.style.left = `${x}px`;
  menu.style.top = `${y}px`;
}

menu.addEventListener('click', ev => {
  const b = ev.target.closest('button');
  if (!b || b.disabled) return;
  if (b.dataset.act === 'move') $('#btn-move-to').click();
  if (b.dataset.act === 'to-arm') $('#btn-cursor-to-arm').click();
  hideMenu();
});

function wireCursorMenu(canvas) {
  canvas.addEventListener('contextmenu', ev => ev.preventDefault());
  canvas.addEventListener('pointerdown', ev => {
    hideMenu();
    if (ev.button === 2) rightDown = { x: ev.clientX, y: ev.clientY };
  });
  canvas.addEventListener('pointerup', ev => {
    if (ev.button !== 2 || !rightDown) return;
    const moved = Math.hypot(ev.clientX - rightDown.x, ev.clientY - rightDown.y);
    rightDown = null;
    if (moved < 5 && viewer?.pickCursor(ev.clientX, ev.clientY)) showMenu(ev.clientX, ev.clientY);
  });
  window.addEventListener('pointerdown', ev => { if (!menu.contains(ev.target) && ev.target !== canvas) hideMenu(); });
  window.addEventListener('keydown', ev => { if (ev.key === 'Escape') hideMenu(); });
}

// ---------- queue ----------

function timeOf(t) {
  return t ? new Date(t * 1000).toLocaleTimeString([], { hour12: false }) : '';
}

function renderQueue(q) {
  queueState = q;
  const list = $('#queue');
  const scroll = list.scrollTop;
  list.textContent = '';
  const jobs = [...q.jobs].sort((a, b) => b.id - a.id);  // newest first: queued, then running, then history
  if (!jobs.length) {
    list.innerHTML = '<li class="empty">Nothing queued yet.</li>';
  }
  for (const j of jobs) {
    const li = document.createElement('li');
    li.className = j.status;
    li.dataset.id = j.id;
    li.title = `${j.status}. Double-click to queue again.`;
    const st = document.createElement('span');
    st.className = 'st';
    st.textContent = STATUS_GLYPH[j.status] || '?';
    const id = document.createElement('span');
    id.className = 'id';
    id.textContent = `#${j.id}`;
    const label = document.createElement('span');
    label.className = 'label';
    label.textContent = j.label;
    li.append(st, id, label);
    if (j.status === 'queued') {
      const x = document.createElement('button');
      x.type = 'button';
      x.className = 'x';
      x.textContent = '×';
      x.title = 'Cancel';
      x.onclick = ev => { ev.stopPropagation(); send({ type: 'cancel_job', id: j.id }); };
      li.append(x);
    } else {
      const when = document.createElement('span');
      when.className = 'when';
      const dur = j.finished && j.started ? ` · ${(j.finished - j.started).toFixed(1)}s` : '';
      when.textContent = j.status === 'running' ? 'running' : `${timeOf(j.finished || j.created)}${dur}`;
      li.append(when);
    }
    if (j.detail) {
      const d = document.createElement('span');
      d.className = 'detail';
      d.textContent = j.detail;
      li.append(d);
    }
    if (j.error) {
      const e = document.createElement('span');
      e.className = 'err';
      e.textContent = j.error;
      li.append(e);
    }
    li.addEventListener('dblclick', () => send({ type: 'rerun', id: j.id }));
    list.append(li);
  }
  list.scrollTop = scroll;

  const queued = q.jobs.filter(j => j.status === 'queued').length;
  $('#queue-count').textContent = queued ? `${queued} waiting` : '';
  const btn = $('#btn-queue-toggle');
  btn.textContent = q.paused ? '▶ Play' : '⏸ Pause';
  btn.classList.toggle('paused', q.paused);
  renderQueueNote();
}

function renderQueueNote() {
  const note = $('#queue-note');
  let text = '';
  if (queueState.paused) text = 'Paused: the running item finishes, nothing new starts.';
  note.hidden = !text;
  note.textContent = text;
}

$('#btn-queue-toggle').onclick = () => send({ type: 'queue_pause', value: !queueState.paused });
$('#btn-queue-clear').onclick = () => send({ type: 'clear_history' });
$('#btn-queue-clear-all').onclick = () => send({ type: 'clear_all' });

// ---------- console ----------

function addLog(entry) {
  const log = $('#log');
  const nearBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 40;
  const div = document.createElement('div');
  div.className = entry.kind;
  const t = document.createElement('span');
  t.className = 't';
  t.textContent = timeOf(entry.t);
  div.append(t, document.createTextNode(entry.text));
  log.append(div);
  while (log.childElementCount > 1500) log.firstElementChild.remove();
  if (nearBottom) log.scrollTop = log.scrollHeight;
}

let history = [];
try { history = JSON.parse(localStorage.getItem('dexarm.history') || '[]'); } catch { history = []; }
let histPos = history.length;

$('#cmd-form').addEventListener('submit', ev => {
  ev.preventDefault();
  const input = $('#cmd');
  const cmd = input.value.trim();
  if (!cmd) return;
  send({ type: 'raw', cmd });
  if (history[history.length - 1] !== cmd) history.push(cmd);
  history = history.slice(-200);
  try { localStorage.setItem('dexarm.history', JSON.stringify(history)); } catch { /* storage unavailable */ }
  histPos = history.length;
  input.value = '';
});

$('#cmd').addEventListener('keydown', ev => {
  if (ev.key === 'ArrowUp' && histPos > 0) {
    histPos--;
    ev.target.value = history[histPos];
    ev.preventDefault();
  } else if (ev.key === 'ArrowDown') {
    histPos = Math.min(history.length, histPos + 1);
    ev.target.value = history[histPos] ?? '';
    ev.preventDefault();
  }
});

$('#cmd').addEventListener('paste', ev => {
  const text = ev.clipboardData.getData('text');
  if (text.includes('\n')) {
    ev.preventDefault();
    send({ type: 'raw', cmd: text });
  }
});

// ---------- buttons ----------

$('#btn-stop').onclick = () => send({ type: 'estop' });
$('#btn-kill').onclick = () => send({ type: 'kill', value: !state?.killed });
$('#btn-home').onclick = () => send({ type: 'raw', cmd: 'M1112' });
$('#motors-toggle').onclick = ev => {
  const on = ev.currentTarget.getAttribute('aria-checked') === 'true';
  send({ type: 'motors', on: !on });
};
$('#btn-disconnect').onclick = () => send({ type: 'disconnect' });
$('#autoconnect').onchange = ev => send({ type: 'autoconnect', value: ev.target.checked });
$('#speed').addEventListener('input', ev => { $('#speed-val').textContent = ev.target.value; });
$('#speed').addEventListener('change', ev => send({ type: 'feedrate', value: ev.target.value }));
for (const [id, key] of [['ease-accel', 'accel'], ['ease-jerk', 'jerk']]) {
  $('#' + id).addEventListener('input', ev => { $('#' + id + '-val').textContent = ev.target.value; });
  $('#' + id).addEventListener('change', ev => send({ type: 'motion', [key]: +ev.target.value }));
}
$('#path-curve').addEventListener('change', ev => send({ type: 'path', curve: ev.target.checked }));
$('#path-style').addEventListener('change', ev => send({ type: 'path', style: ev.target.value }));
$('#z-min').addEventListener('change', ev => {
  const v = parseFloat(ev.target.value);
  if (Number.isFinite(v)) send({ type: 'z_min', value: v });
});
$('#z-min').addEventListener('keydown', ev => { if (ev.key === 'Enter') ev.target.blur(); });
$('#ease-save').onclick = () => send({ type: 'motion', save: true });
function renderLayoutButton(layout) {
  $('#btn-layout').textContent = layout === 'quad' ? '▣ 1 view' : '⊞ 4 views';
}
$('#btn-layout').onclick = () => viewer?.setLayout(viewer.layout === 'quad' ? 'single' : 'quad');

window.addEventListener('keydown', ev => {
  if (ev.key === 'Escape') send({ type: 'estop' });
});

// ---------- start ----------

buildRows();
loadLimits().then(() => state && renderRows(state)).catch(() => {});
setControlsEnabled(false, 'Connecting to the server…');
try {
  viewer = new Viewer($('#viewport'), {
    onStatus: msg => {
      const el = $('#viewer-msg');
      el.hidden = !msg;
      el.textContent = msg || '';
    },
    onCursor: onCursorDrag,
  });
  if (debugPose) {
    const [yaw, shoulder, elbow] = debugPose.split(',').map(Number);
    viewer.setPoses({ yaw, shoulder, elbow }, null);
  }
} catch (e) {
  $('#viewer-msg').textContent = `3D view unavailable: ${e.message}`;
}
window.dexViewer = viewer;  // handy for debugging from the browser console
{
  const box = $('#show-trails');
  try { box.checked = localStorage.getItem('dexarm.showTrails') !== '0'; } catch {}
  const apply = () => {
    viewer?.setTrailVisible(box.checked);
    try { localStorage.setItem('dexarm.showTrails', box.checked ? '1' : '0'); } catch {}
  };
  box.addEventListener('change', apply);
  apply();
}
if (viewer) { viewer.onLayout = renderLayoutButton; renderLayoutButton(viewer.layout); }
viewer?.setReachTest((x, y, z) => reachable(x, y, z, moduleOffset()));  // includes the minimum Z
{
  const box = $('#debug-ghost');
  let on = false;
  try { on = localStorage.getItem('dexarm.debugGhost') === '1'; } catch {}
  const apply = () => {
    viewer?.setGhostEnabled(box.checked);
    $('#legend-ghost').hidden = !box.checked;
    try { localStorage.setItem('dexarm.debugGhost', box.checked ? '1' : '0'); } catch {}
  };
  box.checked = on;
  box.addEventListener('change', apply);
  apply();
}

// ---------- graph ----------

graph = new Graph($('#graph'), {
  onStatus: (text, rec) => {
    const el = $('#graph-status');
    el.textContent = text;
    el.classList.toggle('rec', rec);
  },
});
window.dexGraph = graph;
const chipHost = $('#graph-channels');
CHANNELS.forEach((c, i) => {
  if (i === 3) chipHost.insertAdjacentHTML('beforeend', '<span class="lt-sep" aria-hidden="true"></span>');
  const label = document.createElement('label');
  label.className = 'lt on';
  label.style.setProperty('--c', c.color);
  label.innerHTML = `<input type="checkbox" checked><span class="sw"></span>${c.label}`;
  label.querySelector('input').addEventListener('change', ev => {
    graph.setVisible(c.key, ev.target.checked);
    label.classList.toggle('on', ev.target.checked);
  });
  chipHost.append(label);
});
chipHost.insertAdjacentHTML('beforeend', `<span class="lt-sep" aria-hidden="true"></span>
  <span class="lt on lt-static" style="--c:${COMMAND_COLOR}"><span class="sw sw-dash"></span>Command start</span>`);
$('#graph-motors-off').addEventListener('change', ev => { graph.trackWhenMotorsOff = ev.target.checked; });
$('#graph-movement-only').addEventListener('change', ev => { graph.movementOnly = ev.target.checked; });
{
  const md = $('#graph-min-delta');
  let saved = 1.0;
  try { const v = parseFloat(localStorage.getItem('dexarm.minDelta')); if (Number.isFinite(v) && v >= 0) saved = v; } catch {}
  const apply = () => {
    let v = parseFloat(md.value);
    if (!Number.isFinite(v) || v < 0) v = 0;   // never negative
    md.value = Number.isInteger(v) ? v.toFixed(1) : String(v);
    graph.minDelta = v;
    try { localStorage.setItem('dexarm.minDelta', String(v)); } catch {}
  };
  md.value = String(saved);
  md.addEventListener('change', apply);
  md.addEventListener('keydown', ev => { if (ev.key === 'Enter') md.blur(); });
  apply();
}
$('#trail-visible').addEventListener('change', ev => {
  lastTrail = null;  // don't join across the time it was off
});
$('#graph-clear').onclick = () => { graph.clear(); viewer?.clearTrail(); lastTrail = null; };
function setGraphHold(hold) {
  graph.setHold(hold);
  $('#graph-rec').setAttribute('aria-pressed', String(!hold));
  $('#graph-pause').setAttribute('aria-pressed', String(hold));
}
$('#graph-rec').onclick = () => setGraphHold(false);
$('#graph-pause').onclick = () => setGraphHold(true);
if (viewer) wireCursorMenu(viewer.renderer.domElement);
connectWs();


// ---------- bookmarks ----------
// Named positions kept by the server (bookmarks.yaml). Newest first; double-click moves the arm there.

let bookmarks = [];
let lastZMin;
let bmSelected = null;      // id
let bmSelectNewest = false; // select the next bookmark that appears (after Bookmark Arm/Cursor)
let bmEditing = null;       // id being edited in the editor box
let bmDeleteArmed = null;   // timer while Delete waits for its confirming second click

const fmt1 = v => (Math.round(v * 10) / 10).toFixed(1);
const bmText = xyz => `X${fmt1(xyz[0])}  Y${fmt1(xyz[1])}  Z${fmt1(xyz[2])}`;

function bmProblem(xyz) {
  if (xyz.some(v => !Number.isFinite(v))) return 'Enter a number for X, Y and Z.';
  if (state?.z_min != null && xyz[2] < state.z_min) return `Z is below the minimum Z (${state.z_min} mm).`;
  if (!reachable(xyz[0], xyz[1], xyz[2], moduleOffset())) return 'The arm can’t reach this position.';
  return null;
}

function renderBookmarks(list) {
  if (bmSelectNewest && list.length > bookmarks.length) { bmSelected = list[0].id; bmSelectNewest = false; }
  bookmarks = list;
  if (!bookmarks.some(b => b.id === bmSelected)) bmSelected = null;
  if (bmEditing != null && !bookmarks.some(b => b.id === bmEditing)) closeBmEditor();
  $('#bm-count').textContent = bookmarks.length ? `(${bookmarks.length})` : '';
  const ul = $('#bm-list');
  ul.textContent = '';
  if (!bookmarks.length) {
    ul.innerHTML = '<li class="bm-empty">No bookmarks yet. Use Bookmark Arm or Bookmark Cursor.</li>';
  }
  for (const b of bookmarks) {
    const li = document.createElement('li');
    li.dataset.id = b.id;
    const problem = bmProblem(b.xyz);
    li.classList.toggle('selected', b.id === bmSelected);
    li.classList.toggle('bad', !!problem);
    li.title = problem ? `${problem} Double-click is refused.` : 'Double-click to move the arm here';
    const name = document.createElement('span');
    name.className = 'bm-name';
    name.textContent = b.name;
    const coords = document.createElement('span');
    coords.className = 'bm-coords mono';
    coords.textContent = bmText(b.xyz);
    li.append(name, coords);
    ul.append(li);
  }
  renderBmButtons();
  showBookmarksIn3D();
}

function showBookmarksIn3D() {
  viewer?.setBookmarks(bookmarks, bmSelected, $('#bm-show-all').checked);
}

function renderBmButtons() {
  const has = bmSelected != null;
  $('#bm-edit').disabled = !has;
  $('#bm-rename').disabled = !has;
  $('#bm-delete').disabled = !has;
  $('#bm-add-arm').disabled = !state?.measured;
}

function bmSelect(id) {
  bmSelected = id;
  disarmBmDelete();
  for (const li of $('#bm-list').children) li.classList.toggle('selected', +li.dataset.id === id);
  renderBmButtons();
  showBookmarksIn3D();
}

const bmList = $('#bm-list');
bmList.addEventListener('click', ev => {
  const li = ev.target.closest('li[data-id]');
  if (li && !ev.target.closest('input')) bmSelect(+li.dataset.id);
});
bmList.addEventListener('dblclick', ev => {
  const li = ev.target.closest('li[data-id]');
  if (!li || ev.target.closest('input')) return;
  const b = bookmarks.find(x => x.id === +li.dataset.id);
  if (b) send({ type: 'bm_goto', id: b.id });
});
// preview the bookmark in 3D while hovering it
bmList.addEventListener('mouseover', ev => {
  const li = ev.target.closest('li[data-id]');
  const b = li && bookmarks.find(x => x.id === +li.dataset.id);
  if (b && bmEditing == null) viewer?.setPreview(b.xyz);
});
bmList.addEventListener('mouseleave', () => { if (bmEditing == null) viewer?.setPreview(null); });
bmList.addEventListener('keydown', ev => {
  if (!bookmarks.length) return;
  const i = bookmarks.findIndex(b => b.id === bmSelected);
  if (ev.key === 'ArrowDown' || ev.key === 'ArrowUp') {
    ev.preventDefault();
    const j = Math.max(0, Math.min(bookmarks.length - 1, (i < 0 ? -1 : i) + (ev.key === 'ArrowDown' ? 1 : -1)));
    bmSelect(bookmarks[j].id);
  } else if (ev.key === 'Enter' && i >= 0) {
    send({ type: 'bm_goto', id: bmSelected });
  } else if (ev.key === 'F2' && i >= 0) {
    ev.preventDefault(); startBmRename();
  }
});

{
  const box = $('#bm-show-all');
  try { box.checked = localStorage.getItem('dexarm.showAllBookmarks') !== '0'; } catch {}
  box.addEventListener('change', () => {
    try { localStorage.setItem('dexarm.showAllBookmarks', box.checked ? '1' : '0'); } catch {}
    showBookmarksIn3D();
  });
}
$('#bm-add-arm').onclick = () => { bmSelectNewest = true; send({ type: 'bm_add', source: 'arm' }); };
$('#bm-add-cursor').onclick = () => { bmSelectNewest = true; send({ type: 'bm_add', source: 'cursor' }); };

// --- rename: in place, Enter saves, Esc cancels ---
function startBmRename() {
  const b = bookmarks.find(x => x.id === bmSelected);
  const li = b && bmList.querySelector(`li[data-id="${b.id}"]`);
  if (!li) return;
  const name = li.querySelector('.bm-name');
  const input = document.createElement('input');
  input.value = b.name;
  input.maxLength = 80;
  name.textContent = '';
  name.append(input);
  input.focus(); input.select();
  let done = false;
  const finish = save => {
    if (done) return;
    done = true;
    const v = input.value.trim();
    if (save && v && v !== b.name) send({ type: 'bm_rename', id: b.id, name: v });
    name.textContent = save && v ? v : b.name;
  };
  input.addEventListener('keydown', ev => {
    ev.stopPropagation();
    if (ev.key === 'Enter') finish(true);
    if (ev.key === 'Escape') finish(false);
  });
  input.addEventListener('blur', () => finish(true));
}
$('#bm-rename').onclick = startBmRename;

// --- delete: second click within 3 s confirms ---
function disarmBmDelete() {
  clearTimeout(bmDeleteArmed);
  bmDeleteArmed = null;
  const btn = $('#bm-delete');
  btn.classList.remove('confirm');
  btn.textContent = 'Delete';
}
$('#bm-delete').onclick = () => {
  if (bmSelected == null) return;
  if (!bmDeleteArmed) {
    const btn = $('#bm-delete');
    btn.classList.add('confirm');
    btn.textContent = 'Confirm delete';
    bmDeleteArmed = setTimeout(disarmBmDelete, 3000);
    return;
  }
  send({ type: 'bm_delete', id: bmSelected });
  disarmBmDelete();
};

// --- edit box ---
const bmInputs = ['#bm-x', '#bm-y', '#bm-z'].map(q => $(q));
const bmEditorXYZ = () => bmInputs.map(i => parseFloat(i.value));
function setBmEditor(xyz) { bmInputs.forEach((i, k) => { i.value = fmt1(xyz[k]); }); checkBmEditor(); }
function checkBmEditor() {
  const xyz = bmEditorXYZ();
  const problem = bmProblem(xyz);
  bmInputs.forEach((i, k) => i.classList.toggle('bad', !!problem && (problem.startsWith('Enter') ? !Number.isFinite(xyz[k]) : true)));
  $('#bm-editor-err').textContent = problem || '';
  $('#bm-modify').disabled = !!problem;
  $('#bm-modify').title = problem ? 'Fix the coordinates first' : 'Save these coordinates';
  if (xyz.every(Number.isFinite)) viewer?.setPreview(xyz);
}
function openBmEditor() {
  const b = bookmarks.find(x => x.id === bmSelected);
  if (!b) return;
  bmEditing = b.id;
  $('#bm-editor-name').textContent = `“${b.name}”`;
  $('#bm-editor').hidden = false;
  setBmEditor(b.xyz);
  bmInputs[0].focus();
}
function closeBmEditor() {
  bmEditing = null;
  $('#bm-editor').hidden = true;
  viewer?.setPreview(null);
}
$('#bm-edit').onclick = openBmEditor;
bmInputs.forEach(i => {
  i.addEventListener('input', checkBmEditor);
  i.addEventListener('keydown', ev => {
    if (ev.key === 'Enter' && !$('#bm-modify').disabled) $('#bm-modify').click();
    if (ev.key === 'Escape') closeBmEditor();
  });
});
$('#bm-set-arm').onclick = () => {
  const m = state?.measured;
  if (m) setBmEditor([m.x, m.y, m.z]);
};
$('#bm-set-cursor').onclick = () => { if (state?.cursor) setBmEditor(state.cursor); };
$('#bm-modify').onclick = () => {
  const xyz = bmEditorXYZ();
  if (bmEditing == null || bmProblem(xyz)) return;
  send({ type: 'bm_move', id: bmEditing, xyz });
  closeBmEditor();
};
$('#bm-cancel').onclick = closeBmEditor;
