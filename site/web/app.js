import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import URDFLoader from "urdf-loader";
import { currentLang, setLang, t } from "./i18n.js";
import { HandModel, TIPS } from "./teleop.js";

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
const HAND21 = [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25];
const XR_BONES = [
  "XRHand_Wrist",
  ...["Metacarpal", "Proximal", "Distal", "Tip"].map((b) => `XRHand_Thumb${b}`),
  ...["Index", "Middle", "Ring", "Little"].flatMap((f) => ["Metacarpal", "Proximal", "Intermediate", "Distal", "Tip"]
    .map((b) => `XRHand_${f}${b}`)),
];
const NEUTRAL = 0xa3a7ad;
const SKIN = 0xd8a88a;
const PALM_TO_WORLD = new THREE.Matrix4().set(
  0, 1, 0, 0,
  0, 0, 1, 0,
  1, 0, 0, 0,
  0, 0, 0, 1,
);

const state = {
  view: "intro",
  section: "intro",
  hand: null,
  motion: "pinch_index",
  side: "right",
  human: "beside",
  playing: true,
  posing: false,
  mode: "motion",
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

const css = getComputedStyle(document.documentElement);
const color = (name) => new THREE.Color(css.getPropertyValue(name).trim());

const canvas = $("#view");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.toneMapping = THREE.NeutralToneMapping;
renderer.toneMappingExposure = 0.9;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.7;
pmrem.dispose();
const camera = new THREE.PerspectiveCamera(35, 1, 0.005, 20);
camera.position.set(0.25, 0.18, 0.55);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0, 0.08, 0);

scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f98, 0.5));
const key = new THREE.DirectionalLight(0xffffff, 1.2);
key.position.set(0.6, 1.2, 0.9);
scene.add(key);
const fill = new THREE.DirectionalLight(0xffffff, 0.4);
fill.position.set(-0.8, 0.4, -0.6);
scene.add(fill);
const shadowLight = new THREE.DirectionalLight(0xffffff, 0.5);
shadowLight.castShadow = true;
shadowLight.shadow.mapSize.set(1024, 1024);
shadowLight.shadow.radius = 6;
shadowLight.shadow.bias = -0.0005;
scene.add(shadowLight, shadowLight.target);
const shadowFloor = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.ShadowMaterial({ opacity: 0.22 }));
shadowFloor.rotation.x = -Math.PI / 2;
shadowFloor.receiveShadow = true;
scene.add(shadowFloor);

const grid = new THREE.GridHelper(1, 20, color("--floor"), color("--floor"));
grid.material.transparent = true;
grid.material.opacity = 0.7;
scene.add(grid);

const mount = new THREE.Group();
mount.matrixAutoUpdate = false;
scene.add(mount);

const human = new THREE.Group();
human.visible = false;
scene.add(human);
const humanFrame = new THREE.Group();
human.matrixAutoUpdate = false;
human.add(humanFrame);
const skeleton = new THREE.Group();
humanFrame.add(skeleton);
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
const skinMaterial = new THREE.MeshStandardMaterial({ color: SKIN, roughness: 0.6, metalness: 0 });
const handMeshes = {};
function capOpenings(g) {
  const index = g.index.array;
  const pos = g.attributes.position;
  const weld = new Map();
  const id = new Int32Array(pos.count);
  for (let i = 0; i < pos.count; i++) {
    const key = [pos.getX(i), pos.getY(i), pos.getZ(i)].map((x) => Math.round(x * 1e5)).join(",");
    if (!weld.has(key)) weld.set(key, i);
    id[i] = weld.get(key);
  }
  const edges = new Map();
  for (let t = 0; t < index.length; t += 3) {
    for (let k = 0; k < 3; k++) {
      const a = index[t + k];
      const b = index[t + (k + 1) % 3];
      const key = id[a] < id[b] ? `${id[a]},${id[b]}` : `${id[b]},${id[a]}`;
      edges.set(key, edges.has(key) ? null : [a, b]);
    }
  }
  const rim = [...edges.values()].filter(Boolean);
  if (!rim.length) return g;
  const center = new THREE.Vector3();
  const all = new THREE.Vector3();
  const v = new THREE.Vector3();
  const used = new Set(rim.flat());
  used.forEach((i) => center.add(v.fromBufferAttribute(pos, i)));
  center.divideScalar(used.size);
  for (let i = 0; i < pos.count; i++) all.add(v.fromBufferAttribute(pos, i));
  all.divideScalar(pos.count);
  const normal = center.clone().sub(all).normalize();
  const out = new THREE.BufferGeometry();
  const first = rim[0][0];
  for (const [name, attr] of Object.entries(g.attributes)) {
    const size = attr.itemSize;
    const data = new attr.array.constructor((attr.count + 1) * size);
    data.set(attr.array);
    const extra = name === "position" ? center.toArray() : name === "normal" ? normal.toArray()
      : Array.from(attr.array.slice(first * size, first * size + size));
    data.set(extra, attr.count * size);
    out.setAttribute(name, new THREE.BufferAttribute(data, size, attr.normalized));
  }
  const c = pos.count;
  const tris = new (c + 1 > 65535 ? Uint32Array : Uint16Array)(index.length + rim.length * 3);
  tris.set(index);
  rim.forEach(([a, b], k) => tris.set([b, a, c], index.length + k * 3));
  out.setIndex(new THREE.BufferAttribute(tris, 1));
  return out;
}

const handModels = Promise.all(["left", "right"].map((side) => new GLTFLoader().loadAsync(`assets/hand/${side}.glb`)
  .then((gltf) => {
    const root = gltf.scene;
    root.traverse((o) => {
      if (o.isSkinnedMesh) {
        o.geometry = capOpenings(o.geometry);
        o.material = skinMaterial;
        o.frustumCulled = false;
        o.castShadow = true;
      }
    });
    root.visible = false;
    root.updateMatrixWorld(true);
    const bones = XR_BONES.map((n) => root.getObjectByName(n));
    const palm = root.getObjectByName("XRHand_Palm");
    [palm, ...bones].forEach((bone) => root.attach(bone));
    humanFrame.add(root);
    handMeshes[side] = { root, bones, palm, model: new HandModel(bones, side) };
  })));

const converted = new WeakMap();

function convert(m, make) {
  if (!converted.has(m)) converted.set(m, make(m));
  return converted.get(m);
}

function fromUrdf(m) {
  const plain = !m || (!m.name && !m.map && m.color.getHex() === 0xffffff);
  if (plain) return convert(fromUrdf, () => new THREE.MeshStandardMaterial({ color: NEUTRAL, roughness: 0.5, metalness: 0.1 }));
  return convert(m, () => new THREE.MeshStandardMaterial({
    color: m.color,
    map: m.map,
    roughness: 0.5,
    metalness: 0.1,
    transparent: m.transparent,
    opacity: m.opacity,
    depthWrite: m.depthWrite,
  }));
}

function setStatus(text) {
  $("#status").textContent = text;
}

function showLoading(stats) {
  const el = $("#loading");
  if (!stats) {
    el.hidden = true;
    return;
  }
  const frac = stats.total ? stats.done / stats.total : stats.files ? stats.finished / stats.files : 0;
  const pct = Math.round(frac * 100);
  el.hidden = false;
  el.setAttribute("aria-valuenow", String(pct));
  el.querySelector(".loading-fill").style.width = `${pct}%`;
  el.querySelector(".loading-text").textContent = t("loading.text", { pct, done: stats.finished, files: stats.files, mb: (stats.total / 1e6).toFixed(1) });
  const failed = el.querySelector(".loading-failed");
  failed.hidden = !stats.failed.length;
  failed.textContent = t("loading.failed", { n: stats.failed.length });
}

const meshWorker = new Worker(new URL("mesh-worker.js", import.meta.url), { type: "module" });
const workerJobs = new Map();
meshWorker.onmessage = (e) => {
  const job = workerJobs.get(e.data.load);
  if (job) job(e.data);
};
const hiddenMaterial = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: false });
const buildQueue = [];
const cancelled = new Set();
let loadSeq = 0;
let pending = null;

function drainQueue(budget) {
  const t0 = performance.now();
  while (buildQueue.length && performance.now() - t0 < budget) buildQueue.shift()();
}

function makeTexture(spec) {
  const t = new THREE.Texture(spec.image);
  t.flipY = spec.flipY;
  t.colorSpace = spec.colorSpace;
  t.wrapS = spec.wrapS;
  t.wrapT = spec.wrapT;
  t.repeat.fromArray(spec.repeat);
  t.offset.fromArray(spec.offset);
  t.needsUpdate = true;
  return t;
}

function buildMaterial(spec, manager, base) {
  const standard = spec.kind === "standard";
  const m = new THREE.MeshStandardMaterial({
    color: new THREE.Color().fromArray(spec.color),
    roughness: standard ? spec.roughness : 0.5,
    metalness: standard ? spec.metalness : 0.1,
    opacity: spec.opacity,
    transparent: spec.transparent,
    side: spec.side ?? THREE.FrontSide,
    vertexColors: Boolean(spec.vertexColors),
  });
  if (standard && spec.emissive) m.emissive.fromArray(spec.emissive);
  if (spec.map) m.map = makeTexture(spec.map);
  if (spec.mapFile) {
    m.map = new THREE.TextureLoader(manager).load(base + spec.mapFile);
    m.map.colorSpace = THREE.SRGBColorSpace;
  }
  return m;
}

function buildGeometries(parts) {
  return parts.map((part) => {
    const g = new THREE.BufferGeometry();
    for (const [name, a] of Object.entries(part.attributes)) {
      g.setAttribute(name, new THREE.BufferAttribute(a.array, a.itemSize, a.normalized));
    }
    if (part.index) g.setIndex(new THREE.BufferAttribute(part.index, 1));
    part.groups.forEach((x) => g.addGroup(x.start, x.count, x.materialIndex));
    return g;
  });
}

function buildObject(parts, geoms, urdfMaterial, manager, base) {
  const group = new THREE.Group();
  const meshes = [];
  parts.forEach((part, k) => {
    const mats = part.materials
      ? part.materials.map((spec) => (spec ? buildMaterial(spec, manager, base) : fromUrdf(urdfMaterial)))
      : [fromUrdf(urdfMaterial)];
    const real = part.multi ? mats : mats[0];
    const mesh = new THREE.Mesh(geoms[k], part.multi ? mats.map(() => hiddenMaterial) : hiddenMaterial);
    mesh.userData.real = real;
    new THREE.Matrix4().fromArray(part.matrix).decompose(mesh.position, mesh.quaternion, mesh.scale);
    group.add(mesh);
    meshes.push(mesh);
  });
  return { group, meshes };
}

function meshLoader(load, meshes, manager, stats) {
  if (!meshes) return (path, _, material, done) => done(null);
  const robotsBase = new URL("robots/", location.href).href;
  const sizeOf = (raw) => {
    const u = new URL(raw, location.href).href;
    const rel = decodeURIComponent(u.slice(robotsBase.length));
    const entry = u.startsWith(robotsBase) && meshes.files[rel.slice(rel.indexOf("/") + 1)];
    return entry ? entry[1] : 0;
  };
  const requests = new Map();
  const byReq = [];
  workerJobs.set(load, (msg) => {
    const r = byReq[msg.req];
    if (msg.type === "fetched") {
      stats.done += r.size;
      stats.finished += 1;
      if (!msg.ok) stats.failed.push(r.url);
      stats.report();
      return;
    }
    buildQueue.push(() => {
      if (cancelled.has(load)) return;
      if (!msg.parts) {
        if (!stats.failed.includes(r.url)) stats.failed.push(r.url);
        r.waiters.forEach((w) => w.finish(null));
        return;
      }
      const geoms = buildGeometries(msg.parts);
      r.waiters.forEach((w) => {
        const { group, meshes: built } = buildObject(msg.parts, geoms, w.material, manager, r.base);
        stats.meshes.push(...built);
        w.finish(group);
      });
    });
  });
  meshWorker.postMessage({ type: "start", load, meshes, robotsBase });
  return (path, _, material, done) => {
    const url = new URL(path, location.href).href;
    manager.itemStart(url);
    const waiter = { material, finish: (obj) => { done(obj); manager.itemEnd(url); } };
    if (requests.has(url)) {
      requests.get(url).waiters.push(waiter);
      return;
    }
    const r = { url, size: sizeOf(url), base: url.slice(0, url.lastIndexOf("/") + 1), waiters: [waiter] };
    requests.set(url, r);
    byReq.push(r);
    stats.total += r.size;
    stats.files += 1;
    stats.report();
    meshWorker.postMessage({ type: "mesh", load, req: byReq.length - 1, url });
  };
}

function jointBox(robot) {
  robot.updateMatrixWorld(true);
  const box = new THREE.Box3();
  const v = new THREE.Vector3();
  robot.traverse((o) => {
    if (o.isURDFJoint || o.isURDFLink) box.expandByPoint(o.getWorldPosition(v));
  });
  return box.expandByScalar(0.01);
}

function reveal(robot) {
  if (!robot.userData.meshes || robot.userData.revealed) return Promise.resolve();
  robot.userData.revealed = true;
  const mats = new Map();
  for (const mesh of robot.userData.loaded) {
    mesh.material = mesh.userData.real;
    mesh.castShadow = true;
    for (const m of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
      if (!mats.has(m)) mats.set(m, [m.transparent, m.opacity]);
    }
  }
  mats.forEach((_, m) => {
    m.transparent = true;
    m.opacity = 0;
  });
  return renderer.compileAsync(mount, camera, scene).then(() => {
    const start = performance.now();
    const step = (now) => {
      const t = Math.min((now - start) / 350, 1);
      mats.forEach(([transparent, opacity], m) => {
        m.opacity = opacity * t;
        if (t === 1) {
          m.transparent = transparent;
          m.needsUpdate = true;
        }
      });
      if (t < 1) requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  });
}

function loadRobot(url, meshes) {
  if (robots.has(url)) return robots.get(url);
  if (pending && !pending.complete) {
    meshWorker.postMessage({ type: "cancel", load: pending.load });
    workerJobs.delete(pending.load);
    cancelled.add(pending.load);
    robots.delete(pending.url);
  }
  const load = ++loadSeq;
  pending = { load, url, complete: false };
  const mine = pending;
  const manager = new THREE.LoadingManager();
  const stats = { total: 0, done: 0, files: 0, finished: 0, failed: [], meshes: [] };
  stats.report = () => pending === mine && showLoading(stats);
  const loader = new URDFLoader(manager);
  loader.loadMeshCb = meshLoader(load, meshes, manager, stats);
  loader.packages = { dexhand_description: new URL("robots", location.href).href };
  let robot = null;
  let markReady = null;
  const ready = new Promise((resolve) => { markReady = resolve; });
  const p = new Promise((resolve) => {
    manager.onLoad = () => {
      if (pending !== mine || !robot) {
        if (!robot && pending === mine) {
          robots.delete(url);
          showLoading(null);
          setStatus(t("status.model"));
        }
        return;
      }
      mine.complete = true;
      workerJobs.delete(load);
      meshWorker.postMessage({ type: "cancel", load });
      robot.userData.loaded = stats.meshes;
      robot.userData.failedMeshes = stats.failed;
      markReady(robot);
    };
    loader.load(url, (r) => {
      robot = r;
      robot.traverse((o) => {
        if (o.isURDFVisual && meshes) o.children.forEach((c) => { c.visible = false; });
      });
      robot.userData.meshes = Boolean(meshes);
      resolve({ robot, ready });
    });
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
  shadowLight.target.position.copy(center);
  shadowLight.position.copy(center).add(new THREE.Vector3(0.1, 1, 0.2).multiplyScalar(radius * 3));
  const cam = shadowLight.shadow.camera;
  cam.left = cam.bottom = -radius * 1.2;
  cam.right = cam.top = radius * 1.2;
  cam.near = radius;
  cam.far = radius * 6;
  cam.updateProjectionMatrix();
  shadowFloor.position.set(center.x, box.min.y - 0.0005, center.z);
  shadowFloor.scale.setScalar(radius * 4);
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
  setStatus("");
  setDownloading(true);
  showLoading({ total: 0, done: 0, files: 0, finished: 0, failed: [] });
  const [{ robot, ready }] = await Promise.all([loadRobot(url, data.meshes), handModels]);
  if (token !== loadToken) return;
  mount.clear();
  mount.add(robot);
  mount.matrix.copy(placement(sideData));
  mount.matrixWorldNeedsUpdate = true;
  const joints = sideData.joints || Object.values(robot.joints)
    .filter((j) => j.jointType !== "fixed" && j.type !== "URDFMimicJoint").map((j) => j.name);
  if (sideData.joints) Object.values(robot.joints).forEach((j) => { j.mimicJoints = []; });
  for (const name of joints) robot.setJointValue(name, 0);
  scene.updateMatrixWorld(true);
  const box = !data.meshes ? new THREE.Box3() : robot.userData.revealed ? new THREE.Box3().setFromObject(mount) : jointBox(robot);
  current = { hand, side: state.side, data, sideData, robot, box, joints, values: new Float32Array(joints.length) };
  const span = box.isEmpty() ? 0 : Math.max(box.max.x - box.min.x, box.max.z - box.min.z) * 3;
  grid.scale.setScalar(Math.max(span, 0.3));
  const animated = Boolean(sideData.clips);
  document.querySelectorAll("#motions button, #human button, #play, #scrub")
    .forEach((b) => { b.disabled = !animated; });
  renderInfo();
  applyHuman();
  if (!window.__firstRender) {
    renderer.render(scene, camera);
    window.__firstRender = Math.round(performance.now());
  }
  ready.then(() => {
    if (token !== loadToken) return;
    showLoading(null);
    state.t = 0;
    setDownloading(false);
    const failed = robot.userData.failedMeshes;
    setStatus(failed.length ? t("status.failed", { n: failed.length }) : "");
    reveal(robot).then(() => {
      if (token !== loadToken) return;
      if (data.meshes) {
        if (sideData.clips) applyPose(poseAt(0).q, null);
        scene.updateMatrixWorld(true);
        current.box = new THREE.Box3().setFromObject(mount);
        applyHuman();
      }
      window.__meshesReady = Math.round(performance.now());
      if (state.mode === "interactive") {
        engineConfigure();
        startPosing();
      }
    });
  });
}

function setDownloading(on) {
  state.downloading = on;
  canvas.style.visibility = on ? "hidden" : "";
}

function humanBox() {
  human.updateMatrixWorld(true);
  const model = current.handMesh;
  return model ? new THREE.Box3().setFromObject(model.root, true)
    : new THREE.Box3().setFromObject(boneLines).expandByScalar(0.012);
}

function fingerFrame(mcp, tip, index, pinky, side) {
  const z = tip.clone().sub(mcp).normalize();
  const y = side === "right" ? index.clone().sub(pinky) : pinky.clone().sub(index);
  y.addScaledVector(z, -y.dot(z)).normalize();
  const x = new THREE.Vector3().crossVectors(y, z);
  return new THREE.Matrix4().makeBasis(x, y, z);
}

function humanAlignment(sd, chirality) {
  const open = motions.skeleton[sd.skeleton].fist[0];
  const at = (k) => new THREE.Vector3(open[k * 7 + 4], open[k * 7 + 5], open[k * 7 + 6]).divideScalar(motions.pScale);
  const v = (a) => new THREE.Vector3().fromArray(a);
  const h = { mcp: at(12), tip: at(15), index: at(7), pinky: at(22) };
  const r = { mcp: v(sd.align.mcp), tip: v(sd.align.tip), index: v(sd.align.index), pinky: v(sd.align.pinky) };
  const fr = fingerFrame(r.mcp, r.tip, r.index, r.pinky, chirality);
  const fh = fingerFrame(h.mcp, h.tip, h.index, h.pinky, chirality);
  const rot = fr.multiply(fh.transpose());
  const s = r.tip.distanceTo(r.mcp) / h.tip.distanceTo(h.mcp);
  const m = rot.clone().multiply(new THREE.Matrix4().makeScale(s, s, s));
  const t = r.mcp.clone().sub(h.mcp.clone().applyMatrix4(m));
  m.setPosition(t);
  return new THREE.Matrix4().multiplyMatrices(placement(sd), m);
}

function applyHuman() {
  const sd = current.sideData;
  human.visible = state.human !== "off" && Boolean(sd.clips);
  const box = current.box.clone();
  if (human.visible) {
    const overlay = state.human === "overlay";
    const chirality = sd.skeleton.split("_as_")[1];
    for (const [side, model] of Object.entries(handMeshes)) model.root.visible = side === chirality;
    current.handMesh = handMeshes[chirality];
    skeleton.visible = false;
    skinMaterial.needsUpdate = skinMaterial.transparent !== overlay;
    skinMaterial.transparent = overlay;
    skinMaterial.opacity = overlay ? 0.35 : 1;
    skinMaterial.depthWrite = !overlay;
    const pose = poseAt(0);
    applyPose(pose.q, pose.p);
    const align = sd.align ? humanAlignment(sd, chirality) : new THREE.Matrix4().makeRotationFromQuaternion(new THREE.Quaternion().setFromRotationMatrix(PALM_TO_WORLD)).scale(new THREE.Vector3(sd.scale, sd.scale, sd.scale));
    human.matrix.copy(align);
    if (!overlay && !box.isEmpty()) {
      const shift = box.min.x - 0.25 * (box.max.x - box.min.x) - humanBox().max.x;
      human.matrix.premultiply(new THREE.Matrix4().makeTranslation(shift, 0, 0));
    }
    box.union(humanBox());
  }
  if (box.isEmpty()) box.setFromCenterAndSize(new THREE.Vector3(0, 0.1, 0), new THREE.Vector3(0.2, 0.2, 0.2));
  grid.position.y = box.min.y;
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
  return { q: lerpRow(frames[i], frames[j], a, () => motions.qScale), p: lerpRow(sk[i], sk[j], a, jointScale) };
}

function jointScale(k) {
  return k % 7 < 4 ? motions.rScale : motions.pScale;
}

function lerpRow(r0, r1, a, scale) {
  const out = new Float32Array(r0.length);
  for (let k = 0; k < r0.length; k++) out[k] = (r0[k] + (r1[k] - r0[k]) * a) / scale(k);
  return out;
}

let lastHuman = null;

function applyPose(q, p) {
  const names = current.sideData.joints;
  if (p) lastHuman = p;
  for (let k = 0; k < names.length; k++) current.robot.setJointValue(names[k], q[k]);
  current.values.set(q);
  if (!p) return;
  for (let k = 0; k < 21; k++) {
    tmpVec.fromArray(p, HAND21[k] * 7 + 4);
    tmpMat.makeTranslation(tmpVec.x, tmpVec.y, tmpVec.z);
    dots.setMatrixAt(k, tmpMat);
  }
  dots.instanceMatrix.needsUpdate = true;
  BONES.forEach(([a, b], k) => {
    for (let c = 0; c < 3; c++) {
      bonePos[k * 6 + c] = p[HAND21[a] * 7 + 4 + c];
      bonePos[k * 6 + 3 + c] = p[HAND21[b] * 7 + 4 + c];
    }
  });
  boneGeom.attributes.position.needsUpdate = true;
  boneGeom.computeBoundingSphere();
  boneGeom.computeBoundingBox();
  const model = current.handMesh;
  if (!model) return;
  model.palm.quaternion.set(p[0], p[1], p[2], p[3]).normalize();
  model.palm.position.set(p[4], p[5], p[6]);
  model.bones.forEach((bone, k) => {
    const o = (k + 1) * 7;
    bone.quaternion.set(p[o], p[o + 1], p[o + 2], p[o + 3]).normalize();
    bone.position.set(p[o + 4], p[o + 5], p[o + 6]);
  });
}

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const engine = { worker: null, ready: false, busy: false, dirty: false, id: 0, robotQ: null, solveMs: [], failed: false };
window.__solveMs = engine.solveMs;
const target = new THREE.Mesh(
  new THREE.SphereGeometry(0.006, 16, 12),
  new THREE.MeshBasicMaterial({ color: color("--accent"), depthTest: false, transparent: true, opacity: 0.9 }),
);
target.renderOrder = 20;
target.visible = false;
humanFrame.add(target);
const gapGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const gap = new THREE.Line(gapGeom, new THREE.LineBasicMaterial({ color: color("--accent"), depthTest: false }));
gap.renderOrder = 20;
gap.visible = false;
humanFrame.add(gap);
const gizmo = new TransformControls(camera, canvas);
gizmo.setMode("translate");
gizmo.setSpace("local");
gizmo.setSize(0.8);
scene.add(gizmo.getHelper());
gizmo.addEventListener("dragging-changed", (e) => { controls.enabled = !e.value; });
gizmo.addEventListener("objectChange", () => followTarget());
let finger = 1;
window.__teleop = { target, camera, humanFrame, tip: (f) => tipCanonical(f) };

function interactive() {
  return state.mode === "interactive" && Boolean(current && current.sideData.clips && current.handMesh);
}

function toArmature(v) {
  const m = current.handMesh.model;
  return v.clone().applyMatrix3(m.toCanon.clone().transpose()).add(m.pos[0]);
}

function tipCanonical(f) {
  return current.handMesh.bones[TIPS[f]].position.clone();
}

function placeTarget(reset) {
  const on = interactive() && human.visible;
  target.visible = on;
  gap.visible = on;
  if (!on) {
    gizmo.detach();
    return;
  }
  if (reset) target.position.copy(tipCanonical(finger));
  if (gizmo.object !== target) gizmo.attach(target);
  gizmo.showX = gizmo.showY = gizmo.showZ = true;
  const pos = gapGeom.attributes.position;
  pos.setXYZ(0, target.position.x, target.position.y, target.position.z);
  const tip = tipCanonical(finger);
  pos.setXYZ(1, tip.x, tip.y, tip.z);
  pos.needsUpdate = true;
}

function followTarget() {
  if (!interactive()) return;
  current.handMesh.model.solveTip(finger, toArmature(target.position));
  showHuman();
  placeTarget(false);
}

function selectFinger(f) {
  finger = f;
  document.querySelectorAll("#fingers button").forEach((b) => b.setAttribute("aria-pressed", String(Number(b.dataset.finger) === f)));
  $("#opposition-field").hidden = f !== 0;
  placeTarget(true);
}

function showHuman() {
  const p = current.handMesh.model.canonical();
  applyPose(engine.robotQ || current.values, p);
  engine.dirty = true;
}

function startPosing() {
  state.posing = true;
  state.playing = false;
  playBtn.classList.add("paused");
  playBtn.setAttribute("aria-label", "Play");
  if (lastHuman) current.handMesh.model.fromCanonical(lastHuman);
  $("#opposition").value = String(current.handMesh.model.angles[1].roll || 0);
  engine.robotQ = Float32Array.from(current.values);
  ensureEngine();
  showHuman();
  selectFinger(finger);
}

function stopPosing() {
  state.posing = false;
  placeTarget(false);
}

function engineConfigure() {
  if (!engine.worker || !current || !current.sideData.config) return;
  engine.ready = false;
  engine.busy = false;
  engine.id += 1;
  const sd = current.sideData;
  const id = engine.id;
  fetch("py/wheel.json").then((r) => (r.ok ? r.json() : null)).then((w) => {
    if (!w) {
      showRow("engine", { frac: 0, text: t("engine.nowheel"), failed: true });
      return;
    }
    engine.worker.postMessage({
      type: "init",
      id,
      wheelUrl: new URL(`py/${w.wheel}`, location.href).href,
      urdfUrl: new URL(sd.engineUrdf, location.href).href,
      urdfRel: sd.engineUrdf.replace(/^robots\//, ""),
      config: sd.config,
      side: current.side,
    });
  });
}

const ENGINE_STAGES = {
  pyodide: ["Pyodide", 0, 0.4],
  packages: ["numpy, scipy", 0.4, 0.88],
  wheel: ["whatslab-sdk", 0.88, 0.95],
  engine: ["building engine", 0.95, 1],
};

function showRow(part, { frac, text, failed }) {
  const box = $("#engine-loading");
  const pct = Math.round(Math.min(Math.max(frac, 0), 1) * 100);
  box.hidden = false;
  box.classList.toggle("failed", Boolean(failed));
  box.setAttribute("aria-valuenow", String(pct));
  box.querySelector(".loading-fill").style.width = `${pct}%`;
  box.querySelector(".loading-value").textContent = text;
}

function ensureEngine() {
  if (engine.worker) return;
  engine.worker = new Worker(new URL("engine-worker.js", import.meta.url), { type: "module" });
  engine.started = performance.now();
  showRow("engine", { frac: 0, text: t("engine.starting") });
  engine.worker.onerror = (e) => {
    engine.failed = true;
    showRow("engine", { frac: 0, text: t("engine.failed", { msg: e.message || "engine-worker.js" }), failed: true });
  };
  engine.worker.onmessage = (e) => {
    const msg = e.data;
    if (msg.type === "stage" || msg.type === "progress") {
      const [label, lo, hi] = ENGINE_STAGES[msg.stage];
      const inner = msg.expected ? Math.min(msg.received / msg.expected, 1) : 0;
      const frac = lo + (hi - lo) * inner;
      const mb = (n) => (n / 1e6).toFixed(1);
      showRow("engine", { frac, text: `${Math.round(frac * 100)}% · ${label}${msg.expected ? ` ${mb(msg.received)} / ${mb(msg.expected)} MB` : ""}` });
    } else if (msg.type === "ready" && msg.id === engine.id) {
      engine.ready = true;
      engine.dirty = true;
      window.__engineReady = Math.round(performance.now() - engine.started);
      $("#engine-loading").hidden = true;
    } else if (msg.type === "error") {
      engine.failed = true;
      showRow("engine", { frac: 0, text: t("engine.failed", { msg: msg.message }), failed: true });
    } else if (msg.type === "pose") {
      engine.busy = false;
      if (msg.id !== engine.id || !msg.values || !current) return;
      engine.solveMs.push(msg.values[0]);
      if (engine.solveMs.length > 200) engine.solveMs.shift();
      engine.robotQ = Float32Array.from(msg.values.slice(1));
      if (state.posing) applyPose(engine.robotQ, lastHuman);
    }
  };
  engineConfigure();
}

function pumpEngine() {
  if (!state.posing || !engine.ready || engine.busy || !engine.dirty || !current) return;
  engine.dirty = false;
  engine.busy = true;
  const sd = current.sideData;
  engine.worker.postMessage({ type: "solve", id: engine.id, xr: current.handMesh.model.xr(Boolean(sd.mirrored)) });
}

function pickFinger(e) {
  const rect = canvas.getBoundingClientRect();
  pointer.set(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObject(current.handMesh.root, true).find((h) => h.object.isSkinnedMesh);
  if (!hit) return -1;
  const local = humanFrame.worldToLocal(hit.point.clone());
  const bones = current.handMesh.bones;
  let best = -1;
  let dist = Infinity;
  TIPS.forEach((tip, f) => {
    for (let j = tip - (f === 0 ? 2 : 3); j <= tip; j++) {
      const d = bones[j].position.distanceTo(local);
      if (d < dist) {
        dist = d;
        best = f;
      }
    }
  });
  return best;
}

canvas.addEventListener("pointerdown", (e) => {
  if (!interactive() || gizmo.dragging || gizmo.axis) return;
  const f = pickFinger(e);
  if (f >= 0) selectFinger(f);
});

window.addEventListener("keydown", (e) => {
  if (!interactive() || e.target.closest("input, select, textarea")) return;
  if (e.key >= "1" && e.key <= "5") selectFinger(Number(e.key) - 1);
  if (e.key === "r" || e.key === "R") setPose("open");
});

function setPose(name) {
  if (!interactive()) return;
  if (name === "open") current.handMesh.model.reset();
  else current.handMesh.model.preset(name);
  $("#opposition").value = String(current.handMesh.model.angles[1].roll || 0);
  showHuman();
  placeTarget(true);
}

function setMode(mode) {
  state.mode = mode;
  document.querySelectorAll("#mode-tabs button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.mode === mode)));
  const live = mode === "interactive";
  $("#motion-row").hidden = live;
  $("#transport").hidden = live;
  $("#pose-row").hidden = !live;
  $("#finger-row").hidden = !live;
  if (live && current && current.handMesh) startPosing();
  if (!live) {
    stopPosing();
    state.playing = true;
    playBtn.classList.remove("paused");
    playBtn.setAttribute("aria-label", "Pause");
  }
}

const jointCells = [];
let lastTable = 0;

const KEYWORDS = /\b(import|from|print|True|False|None)\b/g;

function highlight(code) {
  const text = code.textContent;
  const parts = text.split(/("[^"\n]*")/g);
  code.innerHTML = parts.map((part, i) => (i % 2
    ? `<span class="tok-s">${esc(part)}</span>`
    : esc(part).replace(KEYWORDS, '<span class="tok-k">$1</span>'))).join("");
}

function decorateCode(root) {
  root.querySelectorAll("pre.code").forEach((pre) => {
    const code = pre.querySelector("code");
    if (code.dataset.lang !== "sh") highlight(code);
    const button = document.createElement("button");
    button.type = "button";
    button.className = "copy";
    button.textContent = t("copy");
    button.addEventListener("click", () => {
      navigator.clipboard.writeText(code.textContent).then(() => {
        button.textContent = t("copied");
        setTimeout(() => { button.textContent = t("copy"); }, 1500);
      }, () => { button.textContent = t("copy"); });
    });
    const box = document.createElement("div");
    box.className = "codebox";
    const bar = document.createElement("div");
    bar.className = "codebar";
    bar.append(button);
    pre.replaceWith(box);
    box.append(bar, pre);
  });
}

function applyI18n() {
  document.documentElement.lang = currentLang();
  document.title = t("title");
  document.querySelectorAll("[data-i18n]").forEach((el) => { el.textContent = t(el.dataset.i18n); });
  document.querySelectorAll("[data-i18n-html]").forEach((el) => { el.innerHTML = t(el.dataset.i18nHtml); });
  document.querySelectorAll("[data-i18n-list]").forEach((el) => {
    el.innerHTML = t(el.dataset.i18nList).map((item) => `<li>${item}</li>`).join("");
  });
  document.querySelectorAll("[data-i18n-placeholder]").forEach((el) => { el.placeholder = t(el.dataset.i18nPlaceholder); });
  document.querySelectorAll(".copy").forEach((b) => { b.textContent = t("copy"); });
  document.querySelectorAll(".lang button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.lang === currentLang())));
  if (current && state.view === "detail") renderInfo();
}

function renderInfo() {
  const h = current.hand;
  const sd = current.sideData;
  const urdfName = sd.urdf.split("/").pop();
  $("#d-name").textContent = h.name;
  $("#d-meta").textContent = t("meta", { maker: h.maker, dof: h.dof, fingers: h.fingers, license: h.license });
  const sideName = (x) => t(`side.${x}`);
  const mirrored = sd.mirrored
    ? `<p class="note">${t("note.mirrored", { side: sideName(state.side), other: sideName(state.side === "left" ? "right" : "left") })}</p>`
    : "";
  const modelOnly = sd.clips
    ? `<p class="note">${t("note.offline", { config: esc(sd.config) })}</p>`
    : `<p class="note">${t("note.noconfig")}</p>`;
  const notes = h.notes.map((n) => `<a class="mono" href="${esc(n.url)}" rel="noopener">${esc(n.name)}</a>`).join("<br>");
  const unclear = h.licenseUnclear ? `<p class="note">${t("note.unclear")}</p>` : "";
  const meshes = current.data.meshes ? `${(sd.meshBytes / 1e6).toFixed(1)} MB` : t("stat.notshown");
  const snippet = sd.config
    ? `from whatslab.retarget import HandRetargeter\n\nr = HandRetargeter("dex:${sd.config}", side="${current.side}")`
    : "";
  $("#info").innerHTML = `
    <section class="panel stats" aria-label="Quick stats">
      <div class="stat"><b>${h.dof}</b><span>${t("stat.dof")}</span></div>
      <div class="stat"><b>${h.fingers}</b><span>${t("stat.fingers")}</span></div>
      <div class="stat"><b>${meshes}</b><span>${current.data.meshes ? t("stat.mesh", { n: sd.meshFiles }) : t("stat.nomesh")}</span></div>
    </section>
    <section class="panel joints-panel">
      <h2>${t("joints")}</h2>
      <table class="joints"><tbody></tbody></table>
    </section>
    ${snippet ? `<section class="panel"><h2>${t("usehand")}</h2><pre class="code"><code>${esc(snippet)}</code></pre></section>` : ""}
    <section class="panel">
      <h2>${t("source")}</h2>
      <dl>
        <dt>${t("maker")}</dt><dd>${h.website ? `<a href="${esc(h.website)}" rel="noopener">${esc(h.maker)}</a>` : esc(h.maker)}</dd>
        <dt>${t("license")}</dt><dd>${esc(h.license)}</dd>
        <dt>${t("source")}</dt><dd>${h.sourceUrl ? `<a class="mono" href="${esc(h.sourceUrl)}" rel="noopener">${esc(h.source)}</a>` : esc(h.source)}</dd>
        <dt>${t("model")}</dt><dd><a class="mono" href="${esc(h.folder)}" rel="noopener">${esc(urdfName)}</a></dd>
        ${notes ? `<dt>${t("files")}</dt><dd>${notes}</dd>` : ""}
      </dl>
      ${mirrored}${unclear}${modelOnly}
    </section>`;
  decorateCode($("#info"));
  const body = $("#info tbody");
  jointCells.length = 0;
  for (const name of current.joints.slice(0, current.sideData.actuated || current.joints.length)) {
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
  for (const h of catalog.hands) {
    if (q && !`${h.name} ${h.id} ${h.maker} ${h.license}`.toLowerCase().includes(q)) continue;
    const b = document.createElement("button");
    b.type = "button";
    b.className = "hand";
    b.dataset.id = h.id;
    b.setAttribute("aria-selected", String(state.view === "detail" && h.id === state.hand));
    b.innerHTML = `<span class="name"></span><span class="tag">${h.dof} DoF</span>`;
    b.querySelector(".name").textContent = h.name;
    b.title = `${h.name}\n${h.maker}\n${h.license}`;
    list.append(b);
    shown++;
  }
  if (!shown) list.innerHTML = `<p class="empty">${esc(t("empty", { q: filter }))}</p>`;
  $("#hand-count").textContent = String(catalog.hands.length);
}

function markSelected() {
  document.querySelectorAll(".hand").forEach((b) => b.setAttribute("aria-selected", String(state.view === "detail" && b.dataset.id === state.hand)));
  document.querySelectorAll("#motions button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.motion === state.motion)));
  document.querySelectorAll("#side button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.side === state.side)));
  document.querySelectorAll("#human button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.human === state.human)));
  const nav = state.view === "detail" ? "" : state.section;
  document.querySelectorAll(".toc a").forEach((a) => a.setAttribute("aria-current", String(a.dataset.nav === nav)));
}

function detailHash() {
  return `#${state.hand}&motion=${state.motion}&side=${state.side}`;
}

function writeHash() {
  if (state.view !== "detail") return;
  const h = detailHash();
  if (location.hash !== h) history.replaceState(null, "", h);
}

function parseHash() {
  const raw = decodeURIComponent(location.hash.slice(1));
  const [head, ...rest] = raw.split("&");
  const p = new URLSearchParams(rest.join("&"));
  const legacy = new URLSearchParams(raw);
  if (legacy.has("hand")) return { view: "detail", hand: legacy.get("hand"), motion: legacy.get("motion"), side: legacy.get("side") };
  if (handsById.has(head)) return { view: "detail", hand: head, motion: p.get("motion"), side: p.get("side") };
  return { view: "intro", section: head === "about" ? "about" : "intro" };
}

function route() {
  const r = parseHash();
  closeDrawer();
  if (r.view === "intro") {
    state.view = "intro";
    state.section = r.section;
    $("#intro-view").hidden = false;
    $("#detail-view").hidden = true;
    if (state.mode === "interactive") setMode("motion");
    markSelected();
    if (r.section === "about") $("#about").scrollIntoView({ block: "start" });
    else window.scrollTo(0, 0);
    return;
  }
  const changes = { hand: handsById.has(r.hand) ? r.hand : catalog.default };
  if (motions.motions.some((m) => m.id === r.motion)) changes.motion = r.motion;
  if (r.side === "left" || r.side === "right") changes.side = r.side;
  const entering = state.view !== "detail";
  state.view = "detail";
  $("#intro-view").hidden = true;
  $("#detail-view").hidden = false;
  if (entering) window.scrollTo(0, 0);
  Object.assign(state, changes);
  if ("motion" in changes) state.t = 0;
  markSelected();
  const item = document.querySelector(`.hand[data-id="${state.hand}"]`);
  if (item) item.scrollIntoView({ block: "nearest" });
  if (!current || current.hand.id !== state.hand || current.side !== state.side) {
    show();
  }
  writeHash();
}

function select(changes) {
  const reload = ("hand" in changes && changes.hand !== state.hand) || ("side" in changes && changes.side !== state.side);
  Object.assign(state, changes);
  if ("motion" in changes) state.t = 0;
  markSelected();
  writeHash();
  if (reload) show();
}

function setDrawer(open) {
  $("#sidebar").classList.toggle("open", open);
  $("#backdrop").hidden = !open;
  $(".menu-toggle").setAttribute("aria-expanded", String(open));
}

const store = navigator.cookieEnabled ? window.localStorage : null;

function setHandsOpen(open) {
  $("#hands-panel").hidden = !open;
  $(".toc-toggle").setAttribute("aria-expanded", String(open));
  if (store) store.setItem("handsOpen", open ? "1" : "0");
}

function closeDrawer() {
  setDrawer(false);
}

const motionsEl = $("#motions");
for (const m of motions.motions) {
  const b = document.createElement("button");
  b.type = "button";
  b.setAttribute("role", "radio");
  b.dataset.motion = m.id;
  b.dataset.i18n = `motion.${m.id}`;
  b.textContent = t(`motion.${m.id}`);
  motionsEl.append(b);
}

motionsEl.addEventListener("click", (e) => {
  if (!e.target.dataset.motion) return;
  select({ motion: e.target.dataset.motion });
});
$("#poses").addEventListener("click", (e) => e.target.dataset.pose && setPose(e.target.dataset.pose));
$("[data-pose-reset]").addEventListener("click", () => setPose("open"));
$("#fingers").addEventListener("click", (e) => e.target.dataset.finger && selectFinger(Number(e.target.dataset.finger)));
$("#mode-tabs").addEventListener("click", (e) => e.target.dataset.mode && setMode(e.target.dataset.mode));
$("#opposition").addEventListener("input", (e) => {
  if (!interactive()) return;
  current.handMesh.model.setRoll(Number(e.target.value));
  current.handMesh.model.solveTip(0, toArmature(target.position));
  showHuman();
  placeTarget(false);
});
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
  closeDrawer();
  location.hash = `#${b.dataset.id}`;
});
$("#search").addEventListener("input", (e) => renderList(e.target.value));
$(".menu-toggle").addEventListener("click", () => setDrawer(!$("#sidebar").classList.contains("open")));
$(".toc-toggle").addEventListener("click", () => setHandsOpen($("#hands-panel").hidden));
document.querySelectorAll(".toc > a").forEach((a) => a.addEventListener("click", closeDrawer));
$("#backdrop").addEventListener("click", closeDrawer);
window.addEventListener("keydown", (e) => {
  if (e.key === "Escape") closeDrawer();
});
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
window.addEventListener("hashchange", route);
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
  if (current && current.sideData.clips && !state.downloading && !state.posing) {
    const d = duration();
    if (state.playing) state.t = (state.t + dt * state.speed) % (d + 0.6);
    const t = Math.min(state.t, d);
    const pose = poseAt(t);
    applyPose(pose.q, pose.p);
    scrub.value = Math.round((t / d) * 1000);
    timeEl.textContent = `${t.toFixed(2)} s`;
  }
  updateTable(now);
  drainQueue(4);
  pumpEngine();
  if (state.view === "detail" && !state.downloading) {
    controls.update();
    renderer.render(scene, camera);
  }
  requestAnimationFrame(tick);
}

state.hand = catalog.default;
document.querySelectorAll(".page > pre.code, .prose").forEach((el) => decorateCode(el));
applyI18n();
document.querySelector(".lang").addEventListener("click", (e) => {
  if (!e.target.dataset.lang) return;
  setLang(e.target.dataset.lang);
  applyI18n();
  renderList($("#search").value);
});
setHandsOpen(!store || store.getItem("handsOpen") !== "0");
renderList();
route();
requestAnimationFrame(tick);
