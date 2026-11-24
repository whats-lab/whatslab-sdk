import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { MTLLoader } from "three/addons/loaders/MTLLoader.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import URDFLoader from "urdf-loader";

const $ = (s) => document.querySelector(s);
const esc = (v) => String(v).replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);
const BONES = [
  [0, 1], [1, 2], [2, 3], [3, 4],
  [0, 5], [5, 6], [6, 7], [7, 8],
  [0, 9], [9, 10], [10, 11], [11, 12],
  [0, 13], [13, 14], [14, 15], [15, 16],
  [0, 17], [17, 18], [18, 19], [19, 20],
  [5, 9], [9, 13], [13, 17],
];
const PALM_TO_WORLD = new THREE.Matrix4().set(
  0, 1, 0, 0,
  0, 0, 1, 0,
  1, 0, 0, 0,
  0, 0, 0, 1,
);

const state = {
  hand: null,
  motion: "pinch_index",
  side: "right",
  human: "beside",
  mode: "motion",
  playing: true,
  t: 0,
  speed: 1,
};

const [catalog, motions] = await Promise.all([
  fetch("data/hands.json").then((r) => r.json()),
  fetch("data/motions.json").then((r) => r.json()),
]);
const handsById = new Map(catalog.hands.map((h) => [h.id, h]));
const handData = new Map();
const robots = new Map();
let current = null;
let liveApi = null;

const css = getComputedStyle(document.documentElement);
const color = (name) => new THREE.Color(css.getPropertyValue(name).trim());

const canvas = $("#view");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(35, 1, 0.005, 20);
camera.position.set(0.25, 0.18, 0.55);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0, 0.08, 0);

scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f98, 1.6));
const key = new THREE.DirectionalLight(0xffffff, 2.2);
key.position.set(0.6, 1.2, 0.9);
scene.add(key);
const fill = new THREE.DirectionalLight(0xffffff, 0.6);
fill.position.set(-0.8, 0.4, -0.6);
scene.add(fill);

const grid = new THREE.GridHelper(1, 20, color("--floor"), color("--floor"));
grid.material.transparent = true;
grid.material.opacity = 0.7;
scene.add(grid);

const mount = new THREE.Group();
mount.matrixAutoUpdate = false;
scene.add(mount);

const skeleton = new THREE.Group();
scene.add(skeleton);
const bonePos = new Float32Array(BONES.length * 6);
const boneGeom = new THREE.BufferGeometry();
boneGeom.setAttribute("position", new THREE.BufferAttribute(bonePos, 3));
const boneLines = new THREE.LineSegments(boneGeom, new THREE.LineBasicMaterial({ color: color("--accent") }));
skeleton.add(boneLines);
const dots = new THREE.InstancedMesh(
  new THREE.SphereGeometry(0.0035, 12, 8),
  new THREE.MeshStandardMaterial({ color: color("--accent"), roughness: 0.5 }),
  21,
);
skeleton.add(dots);
const tmpMat = new THREE.Matrix4();
const tmpVec = new THREE.Vector3();

function tidyMaterial(m) {
  const src = m && m.color ? m.color : new THREE.Color(0xd4d7dc);
  const c = src.getHex() === 0xffffff ? new THREE.Color(0xd4d7dc) : src.clone();
  return new THREE.MeshStandardMaterial({
    color: c,
    roughness: 0.62,
    metalness: 0.05,
    transparent: m ? m.transparent : false,
    opacity: m && m.transparent ? m.opacity : 1,
  });
}

function tidy(obj, fallback) {
  obj.traverse((o) => {
    if (o.isMesh) {
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      const out = mats.map((m) => tidyMaterial(m && m.color ? m : fallback));
      o.material = Array.isArray(o.material) ? out : out[0];
    }
  });
  return obj;
}

function loadMesh(path, manager, material, done) {
  const fail = (err) => done(null, err);
  const ext = path.split(".").pop().toLowerCase();
  if (ext === "stl") {
    new STLLoader(manager).load(path, (g) => done(tidy(new THREE.Mesh(g), material)), null, fail);
  } else if (ext === "dae") {
    manager.itemStart(path);
    fetch(path)
      .then((r) => r.text())
      .then((text) => {
        const yUp = text.replace(/<up_axis>\s*Z_UP\s*<\/up_axis>/, "<up_axis>Y_UP</up_axis>");
        const dae = new ColladaLoader(manager).parse(yUp, path.slice(0, path.lastIndexOf("/") + 1));
        done(tidy(dae.scene, material));
      })
      .catch(fail)
      .finally(() => manager.itemEnd(path));
  } else if (ext === "glb" || ext === "gltf") {
    new GLTFLoader(manager).load(path, (g) => done(tidy(g.scene, material)), null, fail);
  } else if (ext === "obj") {
    manager.itemStart(path);
    fetch(path)
      .then((r) => r.text())
      .then((text) => {
        const lib = text.match(/^mtllib\s+(.+)$/m);
        const base = path.slice(0, path.lastIndexOf("/") + 1);
        const mtl = lib
          ? new MTLLoader(manager).setPath(base).loadAsync(lib[1].trim()).then((m) => {
            m.preload();
            return m;
          })
          : Promise.resolve(null);
        return mtl.then((m) => {
          const loader = new OBJLoader();
          if (m) loader.setMaterials(m);
          const obj = loader.parse(text);
          done(tidy(obj, m ? null : material));
        });
      })
      .catch(fail)
      .finally(() => manager.itemEnd(path));
  } else {
    fail(new Error(`unsupported mesh ${path}`));
  }
}

function setStatus(text) {
  $("#status").textContent = text;
}

function loadRobot(url) {
  if (robots.has(url)) return robots.get(url);
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.loadMeshCb = loadMesh;
  loader.packages = { dexhand_description: new URL("robots", location.href).href };
  const ready = new Promise((resolve, reject) => {
    let robot = null;
    manager.onProgress = (_, loaded, total) => setStatus(`Loading meshes ${loaded}/${total}`);
    manager.onLoad = () => robot && resolve(robot);
    loader.load(url, (r) => {
      robot = r;
      if (!manager.isLoading) resolve(r);
    }, null, reject);
  });
  manager.isLoading = true;
  manager.onStart = () => { manager.isLoading = true; };
  const p = ready.then((r) => {
    manager.isLoading = false;
    return r;
  });
  robots.set(url, p);
  return p;
}

function loadHandData(id) {
  if (!handData.has(id)) handData.set(id, fetch(`data/hands/${id}.json`).then((r) => r.json()));
  return handData.get(id);
}

function placement(sideData) {
  if (!sideData.palm) return new THREE.Matrix4().makeRotationX(-Math.PI / 2);
  const r = sideData.palm;
  const w = sideData.wrist;
  const rot = new THREE.Matrix4().set(
    r[0][0], r[0][1], r[0][2], 0,
    r[1][0], r[1][1], r[1][2], 0,
    r[2][0], r[2][1], r[2][2], 0,
    0, 0, 0, 1,
  );
  const shift = new THREE.Matrix4().makeTranslation(-w[0], -w[1], -w[2]);
  return new THREE.Matrix4().multiplyMatrices(PALM_TO_WORLD, rot).multiply(shift);
}

function frameCamera(box) {
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const radius = size.length() * 0.5;
  resize();
  const vfov = THREE.MathUtils.degToRad(camera.fov);
  const hfov = 2 * Math.atan(Math.tan(vfov / 2) * camera.aspect);
  const dist = (radius * 0.9) / Math.sin(Math.min(vfov, hfov) / 2);
  const dir = new THREE.Vector3(0.42, 0.28, 1).normalize();
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(dir, dist);
  camera.near = dist / 100;
  camera.far = dist * 20;
  camera.updateProjectionMatrix();
}

let loadToken = 0;

async function show() {
  const token = ++loadToken;
  const hand = handsById.get(state.hand);
  const data = await loadHandData(hand.id);
  const sideData = data.sides[state.side];
  const url = new URL(sideData.urdf, location.href).href;
  setStatus("Loading model");
  const robot = await loadRobot(url);
  if (token !== loadToken) return;
  setStatus("");
  mount.clear();
  mount.add(robot);
  mount.matrix.copy(placement(sideData));
  mount.matrixWorldNeedsUpdate = true;
  const joints = sideData.joints || Object.values(robot.joints)
    .filter((j) => j.jointType !== "fixed" && j.type !== "URDFMimicJoint").map((j) => j.name);
  for (const name of joints) robot.setJointValue(name, 0);
  scene.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(mount);
  current = { hand, side: state.side, data, sideData, robot, box, joints, values: new Float32Array(joints.length) };
  grid.position.y = box.min.y;
  const span = Math.max(box.max.x - box.min.x, box.max.z - box.min.z) * 3;
  grid.scale.setScalar(Math.max(span, 0.3));
  const animated = Boolean(sideData.clips);
  document.querySelectorAll("#motions button, #human button, #play, #scrub")
    .forEach((b) => { b.disabled = !animated; });
  $(".hands-toggle").textContent = hand.name;
  renderInfo();
  applyHuman();
  if (!window.__firstRender) {
    renderer.render(scene, camera);
    window.__firstRender = Math.round(performance.now());
  }
}

function applyHuman() {
  const sd = current.sideData;
  skeleton.visible = state.human !== "off" && Boolean(sd.clips);
  const box = current.box.clone();
  if (skeleton.visible) {
    const overlay = state.human === "overlay";
    const s = sd.scale;
    const pose = poseAt(0);
    applyPose(pose.q, pose.p);
    skeleton.scale.setScalar(s);
    skeleton.position.set(0, 0, 0);
    skeleton.updateMatrixWorld(true);
    const human = new THREE.Box3().setFromObject(boneLines);
    if (!overlay) skeleton.position.x = box.min.x - 0.25 * (box.max.x - box.min.x) - human.max.x;
    for (const o of [boneLines, dots]) {
      o.material.depthTest = !overlay;
      o.renderOrder = overlay ? 10 : 0;
    }
    skeleton.updateMatrixWorld(true);
    box.union(new THREE.Box3().setFromObject(boneLines));
  }
  frameCamera(box);
}

function clipFrames() {
  return current.sideData.clips[state.motion];
}

function duration() {
  return (clipFrames().length - 1) / motions.fps;
}

function poseAt(t) {
  const frames = clipFrames();
  const f = Math.min(t * motions.fps, frames.length - 1);
  const i = Math.floor(f);
  const j = Math.min(i + 1, frames.length - 1);
  const a = f - i;
  const sk = motions.skeleton[current.sideData.skeleton][state.motion];
  return { q: lerpRow(frames[i], frames[j], a, motions.qScale), p: lerpRow(sk[i], sk[j], a, motions.pScale) };
}

function lerpRow(r0, r1, a, scale) {
  const out = new Float32Array(r0.length);
  for (let k = 0; k < r0.length; k++) out[k] = (r0[k] + (r1[k] - r0[k]) * a) / scale;
  return out;
}

function applyPose(q, p) {
  const names = current.sideData.joints;
  for (let k = 0; k < names.length; k++) current.robot.setJointValue(names[k], q[k]);
  current.values.set(q);
  if (!p) return;
  for (let k = 0; k < 21; k++) {
    tmpVec.set(p[k * 3], p[k * 3 + 1], p[k * 3 + 2]).applyMatrix4(PALM_TO_WORLD);
    tmpMat.makeTranslation(tmpVec.x, tmpVec.y, tmpVec.z);
    dots.setMatrixAt(k, tmpMat);
  }
  dots.instanceMatrix.needsUpdate = true;
  BONES.forEach(([a, b], k) => {
    for (let c = 0; c < 3; c++) {
      bonePos[k * 6 + c] = p[a * 3 + c];
      bonePos[k * 6 + 3 + c] = p[b * 3 + c];
    }
  });
  for (let k = 0; k < BONES.length * 2; k++) {
    tmpVec.fromArray(bonePos, k * 3).applyMatrix4(PALM_TO_WORLD).toArray(bonePos, k * 3);
  }
  boneGeom.attributes.position.needsUpdate = true;
  boneGeom.computeBoundingSphere();
}

const jointCells = [];
let lastTable = 0;

function renderInfo() {
  const h = current.hand;
  const sd = current.sideData;
  const info = $("#info");
  const urdfName = sd.urdf.split("/").pop();
  const mirrored = sd.mirrored
    ? `<p class="note">This repository has no ${state.side} model of this hand. The ${state.side === "left" ? "right" : "left"} model is shown${sd.clips ? ", driven by the mirrored motion" : ""}.</p>`
    : "";
  const modelOnly = sd.clips
    ? `<p class="note">Joint angles are precomputed offline with whatslab-sdk DexPilot retargeting (config <span class="mono">${esc(sd.config)}</span>) from synthetic hand motions.</p>`
    : `<p class="note">whatslab-sdk has no retargeting config for this hand yet, so only the model is shown.</p>`;
  const notes = h.notes.map((n) => `<a class="mono" href="${esc(n.url)}" rel="noopener">${esc(n.name)}</a>`).join("<br>");
  const unclear = h.licenseUnclear
    ? `<p class="note">License unclear upstream. Included for non-commercial research use; see the folder's license notes.</p>`
    : "";
  info.innerHTML = `
    <h1>${esc(h.name)}</h1>
    <p class="id mono">${esc(h.id)}</p>
    <dl>
      <dt>Maker</dt><dd>${h.website ? `<a href="${esc(h.website)}" rel="noopener">${esc(h.maker)}</a>` : esc(h.maker)}</dd>
      <dt>DoF</dt><dd>${h.dof}</dd>
      <dt>Fingers</dt><dd>${h.fingers}</dd>
      <dt>License</dt><dd>${esc(h.license)}</dd>
      <dt>Source</dt><dd>${h.sourceUrl ? `<a class="mono" href="${esc(h.sourceUrl)}" rel="noopener">${esc(h.source)}</a>` : esc(h.source)}</dd>
      <dt>Model</dt><dd><a class="mono" href="${esc(h.folder)}" rel="noopener">${esc(urdfName)}</a></dd>
      ${notes ? `<dt>Files</dt><dd>${notes}</dd>` : ""}
    </dl>
    ${mirrored}${unclear}${modelOnly}
    <table class="joints"><caption>Joints</caption><tbody></tbody></table>`;
  const body = info.querySelector("tbody");
  jointCells.length = 0;
  for (const name of current.joints) {
    const tr = document.createElement("tr");
    const a = document.createElement("td");
    const b = document.createElement("td");
    a.textContent = name;
    a.title = name;
    tr.append(a, b);
    body.append(tr);
    const joint = current.robot.joints[name];
    jointCells.push({ cell: b, prismatic: joint && joint.jointType === "prismatic" });
  }
}

function updateTable(now) {
  if (now - lastTable < 80 || !current) return;
  lastTable = now;
  jointCells.forEach(({ cell, prismatic }, k) => {
    const v = current.values[k];
    cell.textContent = prismatic ? `${(v * 1000).toFixed(1)} mm` : `${THREE.MathUtils.radToDeg(v).toFixed(1)}°`;
  });
}

function renderList(filter = "") {
  const list = $("#hand-list");
  const q = filter.trim().toLowerCase();
  list.innerHTML = "";
  let shown = 0;
  for (const group of catalog.groups) {
    const items = catalog.hands.filter((h) => h.group === group
      && (!q || `${h.name} ${h.id} ${h.maker} ${h.license}`.toLowerCase().includes(q)));
    if (!items.length) continue;
    const title = document.createElement("h2");
    title.className = "group-title";
    title.innerHTML = `${esc(group)}<span>${items.length}</span>`;
    list.append(title);
    for (const h of items) {
      const b = document.createElement("button");
      b.type = "button";
      b.className = "hand";
      b.dataset.id = h.id;
      b.setAttribute("aria-selected", String(h.id === state.hand));
      b.innerHTML = `<span class="name"></span><span class="spec"><b>${h.dof}</b> DoF<br>${h.retargeted ? `${h.fingers} fingers` : "model only"}</span><span class="maker"></span>`;
      b.querySelector(".name").textContent = h.name;
      b.querySelector(".maker").textContent = `${h.maker}, ${h.license}`;
      b.title = `${h.name}\n${h.maker}\n${h.license}`;
      list.append(b);
      shown++;
    }
  }
  if (!shown) list.innerHTML = `<p class="empty">No hand matches “${esc(filter)}”. Try a maker name or a license.</p>`;
}

function markSelected() {
  document.querySelectorAll(".hand").forEach((b) => b.setAttribute("aria-selected", String(b.dataset.id === state.hand)));
  document.querySelectorAll("#motions button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.motion === state.motion)));
  document.querySelectorAll("#side button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.side === state.side)));
  document.querySelectorAll("#human button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.human === state.human)));
  document.querySelectorAll(".tabs button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.mode === state.mode)));
}

function writeHash() {
  const h = `#hand=${state.hand}&motion=${state.motion}&side=${state.side}`;
  if (location.hash !== h) history.replaceState(null, "", h);
}

function readHash() {
  const p = new URLSearchParams(location.hash.slice(1));
  const hand = p.get("hand");
  const motion = p.get("motion");
  const side = p.get("side");
  state.hand = handsById.has(hand) ? hand : state.hand || catalog.default;
  if (motions.motions.some((m) => m.id === motion)) state.motion = motion;
  if (side === "left" || side === "right") state.side = side;
}

function select(changes) {
  const reload = ("hand" in changes && changes.hand !== state.hand) || ("side" in changes && changes.side !== state.side);
  Object.assign(state, changes);
  if ("motion" in changes) state.t = 0;
  markSelected();
  writeHash();
  if (reload) {
    show().then(() => liveApi && liveApi.setHand(current));
  }
}

function setMode(mode) {
  state.mode = mode;
  markSelected();
  const live = mode === "live";
  $("#motion-row").hidden = live;
  $("#transport").hidden = live;
  $("#live-panel").hidden = !live;
  if (live && !liveApi) {
    import("./live.js").then((m) => {
      liveApi = m.createLive({
        video: $("#cam"),
        note: $("#live-note"),
        button: $("#live-start"),
        onPose: (q, p) => current && state.mode === "live" && applyPose(q, p),
      });
      liveApi.setHand(current);
    });
  }
  if (!live && liveApi) liveApi.stop();
}

const motionsEl = $("#motions");
for (const m of motions.motions) {
  const b = document.createElement("button");
  b.type = "button";
  b.setAttribute("role", "radio");
  b.dataset.motion = m.id;
  b.textContent = m.label;
  motionsEl.append(b);
}

motionsEl.addEventListener("click", (e) => e.target.dataset.motion && select({ motion: e.target.dataset.motion }));
$("#side").addEventListener("click", (e) => e.target.dataset.side && select({ side: e.target.dataset.side }));
$("#human").addEventListener("click", (e) => {
  if (!e.target.dataset.human) return;
  state.human = e.target.dataset.human;
  markSelected();
  applyHuman();
});
$("#hand-list").addEventListener("click", (e) => {
  const b = e.target.closest(".hand");
  if (!b) return;
  select({ hand: b.dataset.id });
  $("#sidebar").classList.remove("open");
  $(".hands-toggle").setAttribute("aria-expanded", "false");
});
$("#search").addEventListener("input", (e) => renderList(e.target.value));
$(".hands-toggle").addEventListener("click", (e) => {
  const open = $("#sidebar").classList.toggle("open");
  e.currentTarget.setAttribute("aria-expanded", String(open));
});
$(".tabs").addEventListener("click", (e) => e.target.dataset.mode && setMode(e.target.dataset.mode));
const playBtn = $("#play");
playBtn.addEventListener("click", () => {
  state.playing = !state.playing;
  playBtn.classList.toggle("paused", !state.playing);
  playBtn.setAttribute("aria-label", state.playing ? "Pause" : "Play");
});
const scrub = $("#scrub");
scrub.addEventListener("input", () => {
  if (!current) return;
  state.t = (scrub.value / 1000) * duration();
});
$("#speed").addEventListener("change", (e) => { state.speed = Number(e.target.value); });
window.addEventListener("hashchange", () => {
  const before = { hand: state.hand, side: state.side, motion: state.motion };
  readHash();
  const next = { hand: state.hand, side: state.side, motion: state.motion };
  Object.assign(state, before);
  select(next);
});
window.addEventListener("keydown", (e) => {
  if (e.code === "Space" && e.target === document.body) {
    e.preventDefault();
    playBtn.click();
  }
});

function resize() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.floor(w * renderer.getPixelRatio()) || canvas.height !== Math.floor(h * renderer.getPixelRatio())) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}

let last = performance.now();
const timeEl = $("#time");

function tick(now) {
  const dt = Math.min((now - last) / 1000, 0.1);
  last = now;
  resize();
  if (current && current.sideData.clips && state.mode === "motion") {
    const d = duration();
    if (state.playing) state.t = (state.t + dt * state.speed) % (d + 0.6);
    const t = Math.min(state.t, d);
    const pose = poseAt(t);
    applyPose(pose.q, pose.p);
    scrub.value = Math.round((t / d) * 1000);
    timeEl.textContent = `${t.toFixed(2)} s`;
  }
  updateTable(now);
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

readHash();
renderList();
markSelected();
writeHash();
show();
requestAnimationFrame(tick);
