import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import URDFLoader from 'urdf-loader';

// Joint mapping from physical angles (see kinematics.py) to the URDF's joints.
// In the URDF the arm faces -Y with Z up; at joint value 0 the upper arm leans 4.27° forward of
// vertical and the forearm points 22.8° below horizontal.
const SHOULDER_ZERO = Math.atan2(0.15, 0.0112) * 180 / Math.PI;   // 85.73°
const ELBOW_ZERO = Math.atan2(0.058, 0.138) * 180 / Math.PI;      // 22.80°
const D2R = Math.PI / 180;
const JOINT_KEYS = ['yaw', 'shoulder', 'elbow'];
const HOME_POSE = { yaw: 0, shoulder: 53.6, elbow: 53.6 };
const GHOST_THRESHOLD_DEG = 1.0;   // show the commanded pose only when it differs by more than this…
const GHOST_DELAY_MS = 500;        // …for this long, while the arm is standing still
const GHOST_OPACITY = 0.28;
const PATH_MAX_POINTS = 4096;      // planned-route line buffer (a curved route has a few hundred points at most)

// ---- camera views (three's frame: Y up; firmware +X = three -X, firmware +Y forward = three +Z) ----
const FOV = 38;
const VIEW_TARGET = [0, 0.1, 0.16];
const VIEW_DIST = 0.88;
const PRESETS = {                       // direction from the target to the camera
  iso: [0.42, 0.4, -0.66],              // behind-left and above, so the screen's right is the arm's right (+X)
  top: [0, 1, -1e-4],                   // looking down, forward (+Y) up the screen
  bottom: [0, -1, -1e-4],
  left: [1, 0, 0],                      // the arm's left side (firmware -X)
  right: [-1, 0, 0],                    // the arm's right side (firmware +X)
  front: [0, 0, 1],                     // facing the arm from where it reaches
  back: [0, 0, -1],
};
const PRESET_LABELS = { iso: 'Iso', top: 'Top', bottom: 'Btm', left: 'Left', right: 'Right', front: 'Front', back: 'Back' };
const QUAD_DEFAULTS = [['iso', 'persp'], ['top', 'ortho'], ['front', 'ortho'], ['right', 'ortho']];
const SHOULDER_HEIGHT_M = 0.13;    // firmware Z=0 is at shoulder-pivot height
const GROUND_Z_MM = -SHOULDER_HEIGHT_M * 1000;

const BOOKMARK_COLOR = 0x3d7dff;
const COLORS = { tool: 0xff9f1c, target: 0x3ecf8e, preview: 0xd6deea, cursor: 0xffd166, bad: 0xff0000, drag: 0x4aa8ff };
// 3D trail: one colour per command, in a fixed order; moves made by hand (motors off) are grey
const TRAIL_COLORS = [0x3987e5, 0xd95926, 0x199e70, 0xc98500, 0xd55181, 0x9085e9];
const TRAIL_MANUAL = 0x8a94a2;
const TRAIL_MAX_SEGMENTS = 40000;

function applyPose(robot, j) {
  robot.setJointValue('bases', -j.yaw * D2R);   // + yaw = clockwise from above
  robot.setJointValue('B_Rot->In_1', (SHOULDER_ZERO - j.shoulder) * D2R);
  robot.setJointValue('Join->In_2', (j.elbow - ELBOW_ZERO) * D2R);
}

function labelSprite(text, color) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const g = c.getContext('2d');
  g.font = '600 30px system-ui, sans-serif';
  g.fillStyle = color;
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(text, 128, 32);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  sprite.scale.set(0.12, 0.03, 1);
  return sprite;
}

// ---- marker builders (all in firmware millimetres, drawn on top of the arm) ----

const onTop = o => { o.renderOrder = 10; if (o.material) { o.material.depthTest = false; o.material.transparent = true; } return o; };

function segments(points, color, opacity = 1) {
  const g = new THREE.BufferGeometry().setFromPoints(points);
  return onTop(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color, opacity })));
}

function ring(radius, color, opacity = 1) {
  const pts = [];
  for (let i = 0; i <= 48; i++) {
    const a = (i / 48) * Math.PI * 2;
    pts.push(new THREE.Vector3(Math.cos(a) * radius, Math.sin(a) * radius, 0));  // fw XY = horizontal
  }
  return onTop(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color, opacity })));
}

/** 3D reticle: three axis lines through a point, with a small gap at the centre so the point itself stays visible. */
// `dash` splits each arm into pieces of that length; `phase` 0 draws pieces 1, 3, 5..., phase 1 draws 2, 4, 6...,
// so two reticles with opposite phases at the same point add up to one solid line without overlapping.
function reticle(color, { arm = 20, gap = 3, opacity = 1, dash = 0, phase = 0 } = {}) {
  const pts = [];
  for (const axis of [[1, 0, 0], [0, 1, 0], [0, 0, 1]]) {
    const d = new THREE.Vector3(...axis);
    for (const sgn of [1, -1]) {
      if (!dash) { pts.push(d.clone().multiplyScalar(sgn * gap), d.clone().multiplyScalar(sgn * arm)); continue; }
      for (let a = gap, k = 0; a < arm; a += dash, k++) {
        if (k % 2 !== phase) continue;
        pts.push(d.clone().multiplyScalar(sgn * a), d.clone().multiplyScalar(sgn * Math.min(arm, a + dash)));
      }
    }
  }
  return segments(pts, color, opacity);
}
const RETICLE = { arm: 22, gap: 3, dash: 2.5 };  // shared by the cursor and the measured position

function dropLine(color) {
  const line = onTop(new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineDashedMaterial({ color, dashSize: 6, gapSize: 5, opacity: 0.55 })));
  line.update = (from, to) => {
    line.geometry.setFromPoints([from, to]);
    line.computeLineDistances();
  };
  return line;
}

/** Crosshair locator: three axis lines (kept clear of the centre point) and a horizontal ring. */
function locator(color, opacity = 1) {
  const g = new THREE.Group();
  g.add(reticle(color, { arm: 22, gap: 2.5, opacity }));
  g.add(ring(12, color, opacity));
  return g;
}

export class Viewer {
  constructor(container, { onStatus, onCursor } = {}) {
    this.container = container;
    this.onStatus = onStatus;
    this.onCursor = onCursor;       // (xyz, final) when the user drags the cursor
    this.target = { measured: null, commanded: null };
    this.shown = { measured: null, commanded: null };
    this.moving = false;
    this.mismatchSince = null;
    this.toolGoal = null;
    this.toolShown = null;
    this.dragging = false;

    const renderer = this.renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.prepend(renderer.domElement);

    const scene = this.scene = new THREE.Scene();
    scene.background = new THREE.Color(0x14181d);
    scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x20242a, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(0.6, 1.2, 0.8);
    scene.add(sun);
    const fill = new THREE.DirectionalLight(0x9fc4ff, 0.8);
    fill.position.set(-0.8, 0.4, -0.5);
    scene.add(fill);

    scene.add(new THREE.GridHelper(1.0, 20, 0x3a4350, 0x262c35));

    // Firmware axes: +X (robot's right) is three's -X here, +Y (forward) is three's +Z.
    const origin = new THREE.Vector3(0, 0.001, 0);
    scene.add(new THREE.ArrowHelper(new THREE.Vector3(-1, 0, 0), origin, 0.42, 0xff6b6b, 0.03, 0.015));
    scene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), origin, 0.46, 0x5ee08f, 0.03, 0.015));
    const lx = labelSprite('+X  right', '#ff8a8a'); lx.position.set(-0.47, 0.02, 0); scene.add(lx);
    const ly = labelSprite('+Y  forward', '#7ee8a6'); ly.position.set(0, 0.02, 0.51); scene.add(ly);

    this.buildViews();

    this.root = new THREE.Group();
    this.root.rotation.x = -Math.PI / 2;  // URDF is Z-up
    scene.add(this.root);

    // A frame in firmware coordinates: millimetres, X right, Y forward, Z up, origin at shoulder height.
    const fw = this.fw = new THREE.Group();
    fw.position.set(0, SHOULDER_HEIGHT_M, 0);
    fw.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(
      new THREE.Vector3(-1, 0, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 1, 0)));
    fw.scale.setScalar(0.001);
    scene.add(fw);

    this.buildMarkers();
    this.buildTrail();
    this.buildCursor();
    this.buildDragGuide();
    this.loadRobots();

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    renderer.setAnimationLoop(() => this.frame());
  }

  // ---------- views: one or four, each with its own camera, projection and orbit controls ----------

  get camera() { return this.active.camera; }
  get controls() { return this.active.controls; }

  buildViews() {
    const canvas = this.renderer.domElement;
    this.views = [0, 1, 2, 3].map(i => {
      const persp = new THREE.PerspectiveCamera(FOV, 1, 0.01, 20);
      const ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, -10, 20);
      const controls = new OrbitControls(persp, canvas);
      controls.enableDamping = true;
      controls.enabled = false;
      const frame = document.createElement('div');
      frame.className = 'view-frame';
      const bar = document.createElement('div');
      bar.className = 'view-bar';
      const v = { i, persp, ortho, camera: persp, controls, frame, bar, rect: { x: 0, y: 0, w: 1, h: 1 },
                  viewport: new THREE.Vector4() };
      for (const [name, label] of Object.entries(PRESET_LABELS)) {
        const b = document.createElement('button');
        b.type = 'button'; b.textContent = label; b.title = `${name[0].toUpperCase() + name.slice(1)} view`;
        b.onclick = () => this.setPreset(v, name);
        bar.append(b);
      }
      const proj = v.projButton = document.createElement('button');
      proj.type = 'button'; proj.className = 'proj';
      proj.onclick = () => this.setProjection(v, v.camera === v.persp ? 'ortho' : 'persp');
      bar.append(proj);
      frame.append(bar);
      this.container.append(frame);
      return v;
    });
    this.active = this.views[0];
    let saved = 'single';
    try { saved = localStorage.getItem('dexarm.viewLayout') || 'single'; } catch {}
    QUAD_DEFAULTS.forEach(([preset, proj], i) => { this.setProjection(this.views[i], proj); this.setPreset(this.views[i], preset); });
    this.setLayout(saved);

    // the view under the mouse takes the orbit controls, the wheel and the cursor gizmo
    const pick = e => {
      if (this.dragging || e.buttons) return;
      const v = this.viewAt(e.clientX, e.clientY);
      if (v && v !== this.active) this.setActive(v);
    };
    canvas.addEventListener('pointermove', pick, { capture: true });
    canvas.addEventListener('pointerdown', e => { const v = this.viewAt(e.clientX, e.clientY); if (v && v !== this.active && !this.dragging) this.setActive(v); }, { capture: true });
    canvas.addEventListener('wheel', pick, { capture: true, passive: true });
  }

  setLayout(layout) {
    this.layout = layout === 'quad' ? 'quad' : 'single';
    try { localStorage.setItem('dexarm.viewLayout', this.layout); } catch {}
    this.container.classList.toggle('quad', this.layout === 'quad');
    if (this.layout === 'single' || !this.visibleViews().includes(this.active)) this.setActive(this.views[0]);
    this.resize();
    this.onLayout?.(this.layout);
  }

  visibleViews() { return this.layout === 'quad' ? this.views : [this.views[0]]; }

  viewAt(clientX, clientY) {
    const r = this.renderer.domElement.getBoundingClientRect();
    const x = clientX - r.left, y = clientY - r.top;
    return this.visibleViews().find(v => x >= v.rect.x && x < v.rect.x + v.rect.w && y >= v.rect.y && y < v.rect.y + v.rect.h);
  }

  setActive(v) {
    this.active = v;
    for (const w of this.views) {
      w.controls.enabled = w === v && !this.dragging;
      w.frame.classList.toggle('active', w === v && this.layout === 'quad');
    }
    if (this.gizmo) {
      this.gizmo.camera = v.camera;
      this.gizmo.viewport = this.layout === 'quad' ? v.viewport : null;
      this.gizmo.getHelper().updateMatrixWorld(true);
    }
  }

  setPreset(v, name) {
    const dir = new THREE.Vector3(...PRESETS[name]).normalize();
    const target = new THREE.Vector3(...VIEW_TARGET);
    v.controls.target.copy(target);
    v.persp.position.copy(target).addScaledVector(dir, VIEW_DIST);
    v.ortho.position.copy(v.persp.position);
    v.ortho.zoom = 1;
    v.ortho.updateProjectionMatrix();
    v.controls.update();
  }

  /** Switch between perspective and orthographic, keeping the view direction and roughly the same framing. */
  setProjection(v, proj) {
    const target = v.controls.target;
    if (proj === 'ortho' && v.camera === v.persp) {
      const d = v.persp.position.distanceTo(target);
      v.ortho.position.copy(v.persp.position);
      v.ortho.quaternion.copy(v.persp.quaternion);
      v.ortho.zoom = VIEW_DIST / d;
      v.camera = v.ortho;
    } else if (proj === 'persp' && v.camera === v.ortho) {
      const d = VIEW_DIST / v.ortho.zoom;
      const dir = v.ortho.position.clone().sub(target).normalize();
      v.persp.position.copy(target).addScaledVector(dir, d);
      v.camera = v.persp;
    }
    v.controls.object = v.camera;
    v.projButton.textContent = v.camera === v.persp ? 'Persp' : 'Ortho';
    v.projButton.title = v.camera === v.persp ? 'Perspective (click for orthographic)' : 'Orthographic (click for perspective)';
    this.updateProjection(v);
    v.controls.update();
    if (v === this.active && this.gizmo) this.gizmo.camera = v.camera;
  }

  updateProjection(v) {
    const aspect = v.rect.w / v.rect.h;
    v.persp.aspect = aspect;
    v.persp.updateProjectionMatrix();
    const H = 2 * VIEW_DIST * Math.tan(THREE.MathUtils.degToRad(FOV / 2));
    Object.assign(v.ortho, { left: -H * aspect / 2, right: H * aspect / 2, top: H / 2, bottom: -H / 2 });
    v.ortho.updateProjectionMatrix();
  }

  /** Iso view in the active view (the old "Reset view"). */
  resetView() { this.setPreset(this.active, 'iso'); }

  resize() {
    const { clientWidth: w, clientHeight: h } = this.container;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    const g = 1;  // gap between quadrants, px
    const hw = Math.floor((w - g) / 2), hh = Math.floor((h - g) / 2);
    const rects = this.layout === 'quad'
      ? [{ x: 0, y: 0, w: hw, h: hh }, { x: hw + g, y: 0, w: w - hw - g, h: hh },
         { x: 0, y: hh + g, w: hw, h: h - hh - g }, { x: hw + g, y: hh + g, w: w - hw - g, h: h - hh - g }]
      : [{ x: 0, y: 0, w, h }];
    this.views.forEach((v, i) => {
      const r = rects[i];
      v.frame.hidden = !r;
      if (!r) return;
      v.rect = r;
      v.viewport.set(r.x, h - r.y - r.h, r.w, r.h);   // three's viewports count y from the bottom
      Object.assign(v.frame.style, { left: `${r.x}px`, top: `${r.y}px`, width: `${r.w}px`, height: `${r.h}px` });
      this.updateProjection(v);
    });
    this.setActive(this.active);
  }

  // ---------- markers ----------

  buildMarkers() {
    const fw = this.fw;
    // tool point: where the firmware's X/Y/Z actually is (from the encoders)
    this.tool = new THREE.Group();
    // dashed in the opposite phase to the cursor, so where they coincide the dashes interleave instead of overlapping
    this.tool.add(reticle(COLORS.tool, { ...RETICLE, phase: 1 }), ring(9, COLORS.tool, 0.9));
    this.toolDrop = dropLine(COLORS.tool);
    // target of the running / last move
    this.targetLoc = locator(COLORS.target);
    this.targetDrop = dropLine(COLORS.target);
    // preview of where a hovered button / typed value would go
    this.previewLoc = locator(COLORS.preview, 0.85);
    this.previewDrop = dropLine(COLORS.preview);
    // path from tool point to target while moving
    const pathGeo = new THREE.BufferGeometry();
    pathGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(PATH_MAX_POINTS * 3), 3));
    pathGeo.setDrawRange(0, 0);
    this.path = onTop(new THREE.Line(pathGeo,
      new THREE.LineDashedMaterial({ color: COLORS.target, dashSize: 8, gapSize: 5 })));
    for (const o of [this.tool, this.toolDrop, this.targetLoc, this.targetDrop, this.previewLoc, this.previewDrop, this.path]) {
      o.visible = false;
      fw.add(o);
    }
    this.targetXYZ = null;
    this.targetActive = false;
  }

  // ---------- 3D trail ----------

  buildTrail() {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX_SEGMENTS * 6), 3));
    geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX_SEGMENTS * 6), 3));
    geo.setDrawRange(0, 0);
    this.trail = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({
      vertexColors: true, transparent: true, opacity: 0.55, depthWrite: false }));
    this.trail.frustumCulled = false;
    this.trail.renderOrder = 5;
    this.fw.add(this.trail);
    // a dot at every sample, so the line visibly joins real readings (like the timeline)
    const dg = new THREE.BufferGeometry();
    dg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX_SEGMENTS * 3), 3));
    dg.setAttribute('color', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX_SEGMENTS * 3), 3));
    dg.setDrawRange(0, 0);
    this.trailDots = new THREE.Points(dg, new THREE.PointsMaterial({
      size: 3.5, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.85, depthWrite: false }));
    this.trailDots.frustumCulled = false;
    this.trailDots.renderOrder = 5;
    this.fw.add(this.trailDots);
    this.clearTrail();
  }

  clearTrail() {
    this.trailCount = 0;
    this.trailLast = null;
    this.trailGroup = undefined;
    this.trailColorIdx = -1;
    this.trail.geometry.setDrawRange(0, 0);
    this.dotCount = 0;
    this.trailDots.geometry.setDrawRange(0, 0);
  }

  setTrailVisible(on) { this.trail.visible = this.trailDots.visible = on; }

  /** Add a sensed tool position (firmware mm). `group` changes per command ('manual' for hand moves);
   *  `brk` starts a new line instead of connecting to the previous point. */
  addTrail(xyz, group, brk) {
    if (group !== this.trailGroup) {
      this.trailGroup = group;
      if (group !== 'manual') this.trailColorIdx = (this.trailColorIdx + 1) % TRAIL_COLORS.length;
    }
    const p = new THREE.Vector3(...xyz);
    const dotColor = new THREE.Color(group === 'manual' ? TRAIL_MANUAL : TRAIL_COLORS[Math.max(0, this.trailColorIdx)]);
    if (this.dotCount >= TRAIL_MAX_SEGMENTS) this.dropOldestDots();
    {
      const dp = this.trailDots.geometry.attributes.position, dc = this.trailDots.geometry.attributes.color;
      dp.setXYZ(this.dotCount, p.x, p.y, p.z);
      dc.setXYZ(this.dotCount, dotColor.r, dotColor.g, dotColor.b);
      this.dotCount++;
      dp.needsUpdate = dc.needsUpdate = true;
      this.trailDots.geometry.setDrawRange(0, this.dotCount);
    }
    if (this.trailLast && !brk) {
      if (this.trailCount >= TRAIL_MAX_SEGMENTS) this.dropOldestTrail();
      const color = new THREE.Color(group === 'manual' ? TRAIL_MANUAL : TRAIL_COLORS[this.trailColorIdx]);
      const pos = this.trail.geometry.attributes.position, col = this.trail.geometry.attributes.color;
      const i = this.trailCount * 2;
      pos.setXYZ(i, this.trailLast.x, this.trailLast.y, this.trailLast.z);
      pos.setXYZ(i + 1, p.x, p.y, p.z);
      col.setXYZ(i, color.r, color.g, color.b);
      col.setXYZ(i + 1, color.r, color.g, color.b);
      this.trailCount++;
      pos.needsUpdate = col.needsUpdate = true;
      this.trail.geometry.setDrawRange(0, this.trailCount * 2);
    }
    this.trailLast = p;
  }

  dropOldestDots() {
    const drop = Math.floor(TRAIL_MAX_SEGMENTS / 4);
    for (const name of ['position', 'color']) this.trailDots.geometry.attributes[name].array.copyWithin(0, drop * 3);
    this.dotCount -= drop;
  }

  dropOldestTrail() {
    // shift out the oldest quarter so this happens rarely
    const drop = Math.floor(TRAIL_MAX_SEGMENTS / 4);
    for (const name of ['position', 'color']) {
      const a = this.trail.geometry.attributes[name].array;
      a.copyWithin(0, drop * 6);
    }
    this.trailCount -= drop;
  }

  buildCursor() {
    this.cursor = new THREE.Group();
    this.cursorDot = reticle(COLORS.cursor, { ...RETICLE, phase: 0 });
    this.cursorRing = ring(11, COLORS.cursor);
    // invisible, larger sphere so the cursor is easy to hit with a right-click
    this.cursorPick = new THREE.Mesh(new THREE.SphereGeometry(16, 12, 8),
      new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false, depthTest: false }));
    this.cursor.add(this.cursorDot, this.cursorRing, this.cursorPick);
    this.cursor.position.set(0, 250, 0);
    this.cursorDrop = dropLine(COLORS.cursor);
    this.fw.add(this.cursor, this.cursorDrop);

    const tc = this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
    tc.setMode('translate');
    tc.setSpace('local');           // handles follow the firmware axes: red X, green Y, blue Z (up)
    tc.setSize(0.85);
    tc.translationSnap = 1;         // 1 mm; hold Shift for 10 mm
    tc.attach(this.cursor);
    this.scene.add(tc.getHelper());
    // arrows only on the positive end of each axis: drop the negative arrowheads and their grab areas
    const axisIndex = { X: 'x', Y: 'y', Z: 'z' };
    for (const group of [tc._gizmo.gizmo.translate, tc._gizmo.picker.translate]) {
      for (const h of [...group.children]) {
        const k = axisIndex[h.name];
        if (!k) continue;
        // offsets are baked into each handle's geometry, so look at where the geometry sits
        h.geometry.computeBoundingBox();
        const c = h.geometry.boundingBox.getCenter(new THREE.Vector3());
        if (c[k] < -0.01) { group.remove(h); continue; }
        // shafts: start them a gap away from the centre instead of at it (the old centre handle's radius)
        if (group === tc._gizmo.gizmo.translate && h.geometry.boundingBox.max[k] - h.geometry.boundingBox.min[k] > 0.3) {
          const GAP = 0.1, pos = h.geometry.attributes.position, i = { x: 0, y: 1, z: 2 }[k];
          const lo = h.geometry.boundingBox.min[k], hi = h.geometry.boundingBox.max[k];
          for (let n = 0; n < pos.count; n++) {
            const v = pos.getComponent(n, i);
            pos.setComponent(n, i, GAP + (v - lo) / (hi - lo) * (hi - GAP));
          }
          pos.needsUpdate = true;
          h.geometry.computeBoundingBox(); h.geometry.computeBoundingSphere();
        }
      }
    }
    // hide the free-move centre handle but keep its grab area (the picker), so dragging from the centre still works
    for (const h of tc._gizmo.gizmo.translate.children) if (h.name === 'XYZ') h.material = h.material.clone(), h.material.opacity = 0, h.material.visible = false;

    // no white helper lines along the hovered/dragged axis: the reach guides take their place
    for (const h of tc._gizmo.helper.translate.children) h.material = Object.assign(h.material.clone(), { visible: false });
    tc.addEventListener('axis-changed', e => { if (!this.dragging) this.showDragGuide(e.value); });
    tc.addEventListener('dragging-changed', e => {
      if (e.value) this.showDragGuide(tc.axis);
      else requestAnimationFrame(() => this.showDragGuide(tc.axis));  // after release: guide for the new spot, if still hovered
      this.dragging = e.value;
      this.controls.enabled = !e.value;
      this.setCursorReach(this.cursorOk ?? true);
      if (!e.value) this.onCursor?.(this.cursorXYZ(), true);
    });
    let last = 0;
    tc.addEventListener('objectChange', () => {
      const now = performance.now();
      if (now - last > 50) { last = now; this.onCursor?.(this.cursorXYZ(), false); }
    });
    window.addEventListener('keydown', e => { if (e.key === 'Shift') tc.translationSnap = 10; });
    window.addEventListener('keyup', e => { if (e.key === 'Shift') tc.translationSnap = 1; });
  }

  // ---------- drag guides: where the cursor can go along the axis / in the plane being dragged ----------

  /** fn(x, y, z) -> true if the arm may go there (firmware reach + minimum Z), in firmware mm. */
  setReachTest(fn) { this.reachTest = fn; }

  buildDragGuide() {
    this.guide = onTop(new THREE.LineSegments(new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ vertexColors: true, opacity: 0.85 })));
    this.guide.visible = false;
    this.fw.add(this.guide);
  }

  showDragGuide(axis) {
    const ok = this.reachTest;
    if (!ok || !axis || !/^[XYZ]{1,2}$/.test(axis)) { this.guide.visible = false; return; }
    const p0 = [this.cursor.position.x, this.cursor.position.y, this.cursor.position.z];
    // hovering the same handle at the same spot again: reuse the last guide
    const key = `${axis}|${p0.map(v => v.toFixed(1)).join(',')}`;
    if (key === this.guideKey && this.guide.geometry.attributes.position) { this.guide.visible = true; return; }
    this.guideKey = key;
    const pos = [], col = [];
    const GREEN = [0.24, 0.81, 0.45], RED = [1, 0.12, 0.12];
    const seg = (a, b, c) => { pos.push(...a, ...b); col.push(...c, ...c); };
    const K = { X: 0, Y: 1, Z: 2 };
    const RANGE = [[-450, 450], [-30, 450], [-150, 190]];   // a little beyond anything reachable, mm

    if (axis.length === 1) {
      // one axis: green over each reachable stretch of the line, 20 mm red stubs past its ends
      const k = K[axis], STEP = 0.5;
      const at = v => { const p = p0.slice(); p[k] = v; return p; };
      const [lo, hi] = RANGE[k];
      let start = null;
      for (let v = lo; v <= hi + 1e-9; v += STEP) {
        const r = ok(...at(v));
        if (r && start === null) start = v;
        if (start !== null && (!r || v + STEP > hi)) {
          const end = r ? v : v - STEP;
          if (end > start) seg(at(start), at(end), GREEN);
          seg(at(start - 20), at(start), RED);
          seg(at(end), at(end + 20), RED);
          start = null;
        }
      }
    } else {
      // two axes: trace the edge of the reachable area in this plane (marching squares, edges refined by bisection)
      const [ka, kb] = [K[axis[0]], K[axis[1]]];
      const STEP = 3;
      const [a0, a1] = RANGE[ka], [b0, b1] = RANGE[kb];
      const na = Math.ceil((a1 - a0) / STEP), nb = Math.ceil((b1 - b0) / STEP);
      const pt = (a, b) => { const p = p0.slice(); p[ka] = a; p[kb] = b; return p; };
      const inside = new Uint8Array((na + 1) * (nb + 1));
      for (let j = 0; j <= nb; j++) for (let i = 0; i <= na; i++)
        inside[j * (na + 1) + i] = ok(...pt(a0 + i * STEP, b0 + j * STEP)) ? 1 : 0;
      const edgePoint = (pa, pb, qa, qb) => {   // p inside or outside, q the opposite: find the boundary between
        const pin = ok(...pt(pa, pb));
        for (let n = 0; n < 6; n++) {
          const ma = (pa + qa) / 2, mb = (pb + qb) / 2;
          if (ok(...pt(ma, mb)) === pin) { pa = ma; pb = mb; } else { qa = ma; qb = mb; }
        }
        return pt((pa + qa) / 2, (pb + qb) / 2);
      };
      // corners: 0=(i,j) 1=(i+1,j) 2=(i+1,j+1) 3=(i,j+1); edges: 0=c0-c1 1=c1-c2 2=c3-c2 3=c0-c3
      const TABLE = [[], [[3, 0]], [[0, 1]], [[3, 1]], [[1, 2]], [[3, 2], [0, 1]], [[0, 2]], [[3, 2]],
                     [[2, 3]], [[0, 2]], [[0, 3], [1, 2]], [[1, 2]], [[1, 3]], [[0, 1]], [[0, 3]], []];
      for (let j = 0; j < nb; j++) for (let i = 0; i < na; i++) {
        const c = [inside[j * (na + 1) + i], inside[j * (na + 1) + i + 1],
                   inside[(j + 1) * (na + 1) + i + 1], inside[(j + 1) * (na + 1) + i]];
        const code = c[0] | c[1] << 1 | c[2] << 2 | c[3] << 3;
        if (code === 0 || code === 15) continue;
        const A = a0 + i * STEP, B = b0 + j * STEP;
        const corner = [[A, B], [A + STEP, B], [A + STEP, B + STEP], [A, B + STEP]];
        const EDGES = [[0, 1], [1, 2], [3, 2], [0, 3]];
        const ep = e => { const [u, w] = EDGES[e]; return edgePoint(...corner[u], ...corner[w]); };
        for (const [e1, e2] of TABLE[code]) seg(ep(e1), ep(e2), GREEN);
      }
    }
    const g = this.guide.geometry;
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    g.computeBoundingSphere();
    this.guide.visible = pos.length > 0;
  }

  /** True if the screen point (client coordinates) is on the cursor. */
  pickCursor(clientX, clientY) {
    const v = this.viewAt(clientX, clientY);
    if (!v) return false;
    const r = this.renderer.domElement.getBoundingClientRect();
    const x = clientX - r.left - v.rect.x, y = clientY - r.top - v.rect.y;
    const ndc = new THREE.Vector2((x / v.rect.w) * 2 - 1, -(y / v.rect.h) * 2 + 1);
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, v.camera);
    return ray.intersectObject(this.cursorPick, false).length > 0;
  }

  cursorXYZ() {
    const p = this.cursor.position;
    return [+p.x.toFixed(1), +p.y.toFixed(1), +p.z.toFixed(1)];
  }

  /** From the server; ignored while the user is dragging (the drag sets its own colour). */
  setCursor(xyz, unreachable) {
    if (this.dragging) return;
    if (xyz) this.cursor.position.set(...xyz);
    this.setCursorReach(!unreachable);
  }

  /** Yellow when the arm can reach the cursor, red when it can't. */
  setCursorReach(ok) {
    this.cursorOk = ok;
    this.cursorDot.material.color.setHex(ok ? COLORS.cursor : COLORS.bad);
    // ring: red when out of reach, blue while being dragged, otherwise yellow
    this.cursorRing.material.color.setHex(!ok ? COLORS.bad : this.dragging ? COLORS.drag : COLORS.cursor);
  }

  setTool(xyz) { this.toolGoal = xyz ? new THREE.Vector3(...xyz) : null; }

  setTarget(xyz, active, path) {
    this.targetXYZ = xyz ? new THREE.Vector3(...xyz) : null;
    this.targetActive = !!active;
    // planned route (straight or curved) for the dashed line; only used if it ends at this target
    const pts = Array.isArray(path) && path.length > 1 ? path.map(p => new THREE.Vector3(...p)) : null;
    this.targetPath = pts && this.targetXYZ && pts[pts.length - 1].distanceTo(this.targetXYZ) < 0.5 ? pts : null;
  }

  /** The part of the planned route still ahead of point p: p, then the route from the nearest segment on. */
  remainingPath(p, t) {
    const pts = this.targetPath;
    if (!pts) return [p, t];
    let best = 0, bestD = Infinity;
    const seg = new THREE.Line3(), q = new THREE.Vector3();
    for (let i = 0; i < pts.length - 1; i++) {
      seg.set(pts[i], pts[i + 1]).closestPointToPoint(p, true, q);
      const d = q.distanceToSquared(p);
      if (d < bestD) { bestD = d; best = i; }
    }
    return [p, ...pts.slice(best + 1)];
  }

  /** Route preview while dragging the cursor: dashed yellow along the planned route, red straight if blocked. */
  setRoutePreview(pts, ok = true) {
    if (!this.routeLine) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(PATH_MAX_POINTS * 3), 3));
      geo.setDrawRange(0, 0);
      this.routeLine = onTop(new THREE.Line(geo, new THREE.LineDashedMaterial({ color: COLORS.cursor, dashSize: 6, gapSize: 4, opacity: 0.9 })));
      this.routeLine.frustumCulled = false;
      this.fw.add(this.routeLine);
    }
    const line = this.routeLine;
    line.visible = !!(pts && pts.length > 1);
    if (!line.visible) return;
    const pos = line.geometry.attributes.position;
    const n = Math.min(pts.length, PATH_MAX_POINTS);
    for (let i = 0; i < n; i++) pos.setXYZ(i, pts[i][0], pts[i][1], pts[i][2]);
    pos.needsUpdate = true;
    line.geometry.setDrawRange(0, n);
    line.computeLineDistances();
    line.material.color.setHex(ok ? COLORS.cursor : COLORS.bad);
  }

  /** Bookmarks as blue crosshairs: the selected one opaque, the others at 50% when showAll is on. */
  setBookmarks(list, selectedId, showAll) {
    if (!this.bookmarkGroup) { this.bookmarkGroup = new THREE.Group(); this.fw.add(this.bookmarkGroup); }
    const key = JSON.stringify([list.map(b => [b.id, b.xyz]), selectedId, showAll]);
    if (key === this.bookmarkKey) return;
    this.bookmarkKey = key;
    for (const o of [...this.bookmarkGroup.children]) {
      o.traverse(c => { c.geometry?.dispose(); c.material?.dispose(); });
      this.bookmarkGroup.remove(o);
    }
    for (const b of list) {
      const selected = b.id === selectedId;
      if (!selected && !showAll) continue;
      const opacity = selected ? 1 : 0.5;
      const g = new THREE.Group();
      g.add(reticle(BOOKMARK_COLOR, { arm: 18, gap: 3, opacity }), ring(8, BOOKMARK_COLOR, opacity));
      g.position.set(...b.xyz);
      g.traverse(c => { if (c.renderOrder) c.renderOrder = selected ? 9 : 8; });
      this.bookmarkGroup.add(g);
    }
  }

  setPreview(xyz) {
    const on = !!xyz;
    this.previewLoc.visible = this.previewDrop.visible = on;
    if (on) {
      const p = new THREE.Vector3(...xyz);
      this.previewLoc.position.copy(p);
      this.previewDrop.update(p, new THREE.Vector3(p.x, p.y, GROUND_Z_MM));
    }
  }

  // ---------- robots ----------

  loadRobots() {
    const solid = new THREE.MeshStandardMaterial({ color: 0x8d96a3, metalness: 0.35, roughness: 0.55 });
    // polygonOffset pushes the ghost slightly behind the solid arm, so overlapping surfaces don't flicker
    const ghost = this.ghostMaterial = new THREE.MeshStandardMaterial({ color: 0x4aa8ff, transparent: true,
      opacity: 0, depthWrite: false, emissive: 0x0a2a4a, polygonOffset: true, polygonOffsetFactor: 2,
      polygonOffsetUnits: 2 });
    let pending = 2;
    const load = (material, done) => {
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      // Set the material on each mesh as it arrives, rather than waiting for the manager's onLoad.
      // Without this, the second copy could keep the loader's default opaque white material.
      loader.loadMeshCb = (path, mgr, _urdfMaterial, onDone) => loader.defaultMeshLoader(path, mgr, material, (obj, err) => {
        obj?.traverse(o => { if (o.isMesh) o.material = material; });
        onDone(obj, err);
      });
      manager.onLoad = () => { if (--pending === 0) this.onStatus?.(null); };
      manager.onError = url => this.onStatus?.(`Couldn't load ${url}`);
      loader.load('/urdf/dexarm.urdf', robot => {
        // some joints (e.g. the wrist mimic) have no <limit> in the URDF, which the loader reads as 0..0
        Object.values(robot.joints).forEach(j => { j.ignoreLimits = true; });
        applyPose(robot, HOME_POSE);  // sensible pose until the first reading arrives
        done(robot);
      });
    };
    load(solid, r => { this.measuredRobot = r; this.root.add(r); });
    load(ghost, r => { this.commandedRobot = r; r.visible = false; r.renderOrder = 1; this.root.add(r); });
  }

  /** poses: {yaw, shoulder, elbow} in degrees (or null when unknown) */
  setPoses(measured, commanded, moving = false) {
    const ok = p => p && JOINT_KEYS.every(k => typeof p[k] === 'number');
    this.target.measured = ok(measured) ? { ...measured } : null;
    this.target.commanded = ok(commanded) ? { ...commanded } : null;
    this.moving = moving;
  }

  // ---------- per frame ----------

  frame() {
    for (const which of ['measured', 'commanded']) {
      const tgt = this.target[which];
      const robot = which === 'measured' ? this.measuredRobot : this.commandedRobot;
      if (!tgt || !robot) continue;
      const cur = this.shown[which] ?? { ...tgt };
      // ease toward the latest reading so 4 Hz updates look smooth
      for (const k of JOINT_KEYS) cur[k] += (tgt[k] - cur[k]) * 0.25;
      this.shown[which] = cur;
      applyPose(robot, cur);
    }
    this.updateGhost();
    this.updateMarkers();
    const R = this.renderer, quad = this.layout === 'quad';
    R.setScissorTest(quad);
    for (const v of this.visibleViews()) {
      v.controls.update();
      R.setViewport(v.viewport); R.setScissor(v.viewport);
      if (this.gizmo) this.gizmo.camera = v.camera;   // the gizmo sizes and orients itself per camera
      R.render(this.scene, v.camera);
    }
    // leave the gizmo's handles and drag plane oriented for the active view, where the mouse will grab them
    if (this.gizmo) { this.gizmo.camera = this.active.camera; this.gizmo.getHelper().updateMatrixWorld(true); }
  }

  /** Debug view: show the firmware's commanded pose as a ghost when it differs from the encoders. Off by default. */
  setGhostEnabled(on) { this.ghostEnabled = !!on; }

  updateGhost() {
    if (!this.commandedRobot) return;
    // The ghost flags an arm that stopped somewhere other than commanded (e.g. a stall).
    // During a move the two always differ, so it stays hidden then.
    const m = this.target.measured, c = this.target.commanded;
    const differs = !!(m && c && JOINT_KEYS.some(k => Math.abs(m[k] - c[k]) > GHOST_THRESHOLD_DEG));
    const now = performance.now();
    if (!differs || this.moving) this.mismatchSince = null;
    else if (this.mismatchSince === null) this.mismatchSince = now;
    const want = this.ghostEnabled && this.mismatchSince !== null && now - this.mismatchSince > GHOST_DELAY_MS
      ? GHOST_OPACITY : 0;
    const mat = this.ghostMaterial;
    mat.opacity += (want - mat.opacity) * 0.15;
    if (want === 0 && mat.opacity < 0.01) mat.opacity = 0;
    this.commandedRobot.visible = mat.opacity > 0;
  }

  updateMarkers() {
    const ground = p => new THREE.Vector3(p.x, p.y, GROUND_Z_MM);
    // tool point, eased like the arm
    if (this.toolGoal) {
      if (!this.toolShown) this.toolShown = this.toolGoal.clone();
      this.toolShown.lerp(this.toolGoal, 0.25);
      this.tool.position.copy(this.toolShown);
      this.toolDrop.update(this.toolShown, ground(this.toolShown));
    }
    this.tool.visible = this.toolDrop.visible = !!this.toolGoal;

    const t = this.targetXYZ;
    this.targetLoc.visible = this.targetDrop.visible = !!t;
    if (t) {
      this.targetLoc.position.copy(t);
      this.targetDrop.update(t, ground(t));
    }
    const showPath = !!(t && this.toolShown && this.targetActive && this.toolShown.distanceTo(t) > 3);
    this.path.visible = showPath;
    if (showPath) {
      const pts = this.remainingPath(this.toolShown, t).slice(0, PATH_MAX_POINTS);
      const pos = this.path.geometry.attributes.position;
      pts.forEach((q, i) => pos.setXYZ(i, q.x, q.y, q.z));
      pos.needsUpdate = true;
      this.path.geometry.setDrawRange(0, pts.length);
      this.path.geometry.computeBoundingSphere();
      this.path.computeLineDistances();
    }
    const c = this.cursor.position;
    this.cursorDrop.update(c, ground(c));
  }
}
