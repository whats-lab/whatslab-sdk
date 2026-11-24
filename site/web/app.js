import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { MTLLoader } from "three/addons/loaders/MTLLoader.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { toCreasedNormals } from "three/addons/utils/BufferGeometryUtils.js";
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
const HAND21 = [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25];
const XR_BONES = [
  "wrist",
  "thumb-metacarpal", "thumb-phalanx-proximal", "thumb-phalanx-distal", "thumb-tip",
  ...["index", "middle", "ring", "pinky"].flatMap((f) => ["metacarpal", "phalanx-proximal", "phalanx-intermediate", "phalanx-distal", "tip"]
    .map((b) => `${f}-finger-${b}`)),
];
const CREASE = THREE.MathUtils.degToRad(35);
const NEUTRAL = 0xa3a7ad;
const SKIN = 0xd8a88a;
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
humanFrame.quaternion.setFromRotationMatrix(PALM_TO_WORLD);
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
const handModels = Promise.all(["left", "right"].map((side) => new GLTFLoader().loadAsync(`assets/generic-hand/${side}.glb`)
  .then((gltf) => {
    const root = gltf.scene;
    root.traverse((o) => {
      if (o.isSkinnedMesh) {
        o.material = skinMaterial;
        o.frustumCulled = false;
        o.castShadow = true;
      }
    });
    root.visible = false;
    humanFrame.add(root);
    handMeshes[side] = { root, bones: XR_BONES.map((n) => root.getObjectByName(n)) };
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

function embedded(m, linear) {
  if (m.isMeshStandardMaterial) return m;
  return convert(m, () => new THREE.MeshStandardMaterial({
    color: linear ? THREE.ColorManagement.workingToColorSpace(m.color.clone(), THREE.SRGBColorSpace) : m.color,
    map: m.map,
    vertexColors: m.vertexColors,
    roughness: 0.5,
    metalness: 0.1,
    transparent: m.transparent,
    opacity: m.opacity,
    side: m.side,
  }));
}

function faceted(g) {
  const n = g.attributes.normal;
  if (!n) return true;
  if (g.index) return false;
  const tris = n.count / 3;
  const step = Math.max(1, Math.floor(tris / 300));
  let flat = 0;
  let seen = 0;
  for (let t = 0; t < tris; t += step) {
    const i = t * 3;
    seen++;
    const same = (j) => n.getX(i) === n.getX(j) && n.getY(i) === n.getY(j) && n.getZ(i) === n.getZ(j);
    if (same(i + 1) && same(i + 2)) flat++;
  }
  return flat > seen * 0.9;
}

function tidy(obj, urdfMaterial, own, linear) {
  obj.traverse((o) => {
    if (!o.isMesh) return;
    o.castShadow = true;
    if (faceted(o.geometry)) o.geometry = toCreasedNormals(o.geometry, CREASE);
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    const out = mats.map((m) => (own && m ? embedded(m, linear) : fromUrdf(urdfMaterial)));
    o.material = Array.isArray(o.material) ? out : out[0];
  });
  return obj;
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
  el.querySelector(".loading-text").textContent = stats.total
    ? `${pct}% · ${stats.finished}/${stats.files} files · ${(stats.total / 1e6).toFixed(1)} MB`
    : `${pct}% · ${stats.finished}/${stats.files} files`;
  const failed = el.querySelector(".loading-failed");
  failed.hidden = !stats.failed.length;
  failed.textContent = `${stats.failed.length} failed`;
}

function meshLoader(meshes, signal, manager, stats) {
  if (!meshes) return (path, _, material, done) => done(null);
  const robotsBase = new URL("robots/", location.href).href;
  const resolveUrl = (u) => {
    if (!u.startsWith(robotsBase)) return u;
    const rel = decodeURIComponent(u.slice(robotsBase.length));
    const entry = meshes.files[rel.slice(rel.indexOf("/") + 1)];
    return entry ? meshes.base + entry[0].split("/").map(encodeURIComponent).join("/") : u;
  };
  const sizeOf = (u) => {
    const rel = decodeURIComponent(u.slice(robotsBase.length));
    const entry = u.startsWith(robotsBase) && meshes.files[rel.slice(rel.indexOf("/") + 1)];
    return entry ? entry[1] : 0;
  };
  manager.setURLModifier(resolveUrl);
  const get = (path, kind) => {
    const size = sizeOf(path);
    stats.total += size;
    stats.files += 1;
    stats.report();
    return fetch(resolveUrl(path), { signal, cache: "force-cache" })
      .then((r) => (r.ok ? r[kind]() : null))
      .then((body) => body, () => null)
      .then((body) => {
        stats.done += size;
        stats.finished += 1;
        if (!body && !signal.aborted) stats.failed.push(path);
        stats.report();
        return body;
      });
  };
  return (path, _, material, done) => {
    const ext = path.split(".").pop().toLowerCase();
    const base = path.slice(0, path.lastIndexOf("/") + 1);
    const finish = (obj) => {
      done(obj);
      manager.itemEnd(path);
    };
    manager.itemStart(path);
    get(path, ext === "dae" || ext === "obj" ? "text" : "arrayBuffer").then((body) => {
      if (!body) {
        finish(null);
      } else if (ext === "stl") {
        finish(tidy(new THREE.Mesh(new STLLoader().parse(body)), material, false));
      } else if (ext === "dae") {
        const yUp = body.replace(/<up_axis>\s*Z_UP\s*<\/up_axis>/, "<up_axis>Y_UP</up_axis>");
        finish(tidy(new ColladaLoader(manager).parse(yUp, base).scene, material, true, true));
      } else if (ext === "glb" || ext === "gltf") {
        new GLTFLoader(manager).parse(body, base, (g) => finish(tidy(g.scene, material, true)), () => {
          stats.failed.push(path);
          finish(null);
        });
      } else if (ext === "obj") {
        const lib = body.match(/^mtllib\s+(.+)$/m);
        const mtl = lib ? get(base + lib[1].trim(), "text") : Promise.resolve(null);
        mtl.then((text) => {
          const loader = new OBJLoader();
          const m = text && new MTLLoader(manager).parse(text, base);
          if (m) {
            m.preload();
            loader.setMaterials(m);
          }
          finish(tidy(loader.parse(body), material, Boolean(m), true));
        });
      } else {
        stats.failed.push(path);
        finish(null);
      }
    });
  };
}

let pending = null;
const kinematicMaterial = new THREE.MeshStandardMaterial({ color: NEUTRAL, roughness: 0.5, metalness: 0.1 });
const UP = new THREE.Vector3(0, 1, 0);

function addKinematics(robot) {
  const r = 0.0035;
  const ball = new THREE.SphereGeometry(r * 1.6, 16, 12);
  const rod = new THREE.CylinderGeometry(r, r, 1, 12);
  for (const joint of Object.values(robot.joints)) {
    const sphere = new THREE.Mesh(ball, kinematicMaterial);
    sphere.castShadow = true;
    joint.add(sphere);
    const len = joint.position.length();
    if (len < 1e-6) continue;
    const bar = new THREE.Mesh(rod, kinematicMaterial);
    bar.castShadow = true;
    bar.scale.set(1, len, 1);
    bar.position.copy(joint.position).multiplyScalar(0.5);
    bar.quaternion.setFromUnitVectors(UP, joint.position.clone().normalize());
    joint.parent.add(bar);
  }
}

function loadRobot(url, meshes) {
  if (robots.has(url)) return robots.get(url);
  if (pending) pending.abort();
  const controller = new AbortController();
  pending = controller;
  const manager = new THREE.LoadingManager();
  const stats = { total: 0, done: 0, files: 0, finished: 0, failed: [] };
  stats.report = () => controller.signal.aborted || showLoading(stats);
  const loader = new URDFLoader(manager);
  loader.loadMeshCb = meshLoader(meshes, controller.signal, manager, stats);
  loader.packages = { dexhand_description: new URL("robots", location.href).href };
  const p = new Promise((resolve) => {
    let robot = null;
    manager.onLoad = () => {
      if (controller.signal.aborted || !robot) {
        robots.delete(url);
        if (!robot && !controller.signal.aborted) {
          showLoading(null);
          setStatus("The model could not be loaded.");
        }
        return;
      }
      if (!meshes) addKinematics(robot);
      robot.userData.meshBytes = stats.done;
      robot.userData.failedMeshes = stats.failed;
      resolve(robot);
    };
    loader.load(url, (r) => { robot = r; });
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
  showLoading({ total: 0, done: 0, files: 0, finished: 0, failed: [] });
  const [robot] = await Promise.all([loadRobot(url, data.meshes), handModels]);
  if (token !== loadToken) return;
  showLoading(null);
  const failed = robot.userData.failedMeshes;
  setStatus(failed.length ? `${failed.length} of the model's meshes could not be downloaded and are not shown.` : "");
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

function humanBox() {
  human.updateMatrixWorld(true);
  return new THREE.Box3().setFromObject(boneLines).expandByScalar(0.012 * human.scale.x);
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
    skeleton.visible = overlay;
    skinMaterial.transparent = overlay;
    skinMaterial.opacity = overlay ? 0.25 : 1;
    skinMaterial.depthWrite = !overlay;
    const pose = poseAt(0);
    applyPose(pose.q, pose.p);
    human.scale.setScalar(sd.scale);
    human.position.set(0, 0, 0);
    if (!overlay) human.position.x = box.min.x - 0.25 * (box.max.x - box.min.x) - humanBox().max.x;
    for (const o of [boneLines, dots]) {
      o.material.depthTest = !overlay;
      o.renderOrder = overlay ? 10 : 0;
    }
    box.union(humanBox());
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

function applyPose(q, p) {
  const names = current.sideData.joints;
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
  model.bones.forEach((bone, k) => {
    const o = (k + 1) * 7;
    bone.quaternion.set(p[o], p[o + 1], p[o + 2], p[o + 3]).normalize();
    bone.position.set(p[o + 4], p[o + 5], p[o + 6]);
  });
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
    ${mirrored}${unclear}${current.data.meshes ? "" : `<p class="note">Meshes not shown: proprietary mesh license (see MESHES-LICENSE).</p>`}${modelOnly}
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
  for (const h of catalog.hands) {
    if (q && !`${h.name} ${h.id} ${h.maker} ${h.license}`.toLowerCase().includes(q)) continue;
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
