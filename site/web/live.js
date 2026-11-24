import * as THREE from "three";

const TASKS = "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@1.1.1";
const MODEL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task";
const FINGERS = [[6, 5], [11, 9], [16, 13], [21, 17]];
const META = 0.35;

const THUMB_HINT = [0, 0.951, -0.308];

const v = (a) => new THREE.Vector3(a.x, a.y, a.z);

function frame(z, xHint) {
  const x = xHint.clone().addScaledVector(z, -xHint.dot(z)).normalize();
  const y = new THREE.Vector3().crossVectors(z, x);
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
}

export function landmarksToXr(world, side) {
  const P = world.map(v);
  const pos = new Array(26);
  pos[1] = P[0];
  pos[0] = P[0].clone().lerp(P[9], 0.5);
  [2, 3, 4, 5].forEach((j, k) => { pos[j] = P[k + 1]; });
  for (const [base, mcp] of FINGERS) {
    pos[base] = P[0].clone().lerp(P[mcp], META);
    for (let k = 0; k < 4; k++) pos[base + 1 + k] = P[mcp + k];
  }
  const across = side === "right" ? pos[22].clone().sub(pos[7]) : pos[7].clone().sub(pos[22]);
  across.normalize();
  const back = pos[1].clone().sub(pos[12]).normalize();
  const wx = across.clone().addScaledVector(back, -across.dot(back)).normalize();
  const wy = new THREE.Vector3().crossVectors(back, wx);
  const thumbHint = wx.clone().multiplyScalar(THUMB_HINT[0])
    .addScaledVector(wy, THUMB_HINT[1]).addScaledVector(back, THUMB_HINT[2]);
  const quat = new Array(26);
  quat[1] = frame(back, across);
  quat[0] = quat[1].clone();
  const chains = [[[2, 3, 4, 5], thumbHint], ...FINGERS.map(([b]) => [[b, b + 1, b + 2, b + 3, b + 4], wx])];
  for (const [chain, hint] of chains) {
    const fwd = chain.slice(1).map((j, k) => pos[j].clone().sub(pos[chain[k]]).normalize());
    const axis = hint.clone().multiplyScalar(0.1);
    for (let k = 0; k + 1 < fwd.length; k++) axis.add(new THREE.Vector3().crossVectors(fwd[k + 1], fwd[k]));
    chain.forEach((j, k) => {
      quat[j] = frame(fwd[Math.min(k, fwd.length - 1)].clone().negate(), axis);
    });
  }
  const out = new Float64Array(26 * 8);
  for (let j = 0; j < 26; j++) {
    out.set([quat[j].x, quat[j].y, quat[j].z, quat[j].w, pos[j].x, pos[j].y, pos[j].z, 0.01], j * 8);
  }
  return out;
}

export function createLive({ video, note, button, onPose }) {
  let landmarker = null;
  let worker = null;
  let stream = null;
  let hand = null;
  let ready = false;
  let busy = false;
  let lastTime = -1;
  let running = false;
  let solveMs = 0;
  let config = 0;

  const wheel = fetch("py/wheel.json").then((r) => (r.ok ? r.json() : null)).catch(() => null);

  const say = (text) => { note.textContent = text; };

  function configure() {
    if (!worker || !hand || !hand.sideData.config) return;
    ready = false;
    config += 1;
    const sd = hand.sideData;
    const id = config;
    wheel.then((w) => w && worker.postMessage({
      type: "init",
      id,
      wheelUrl: new URL(`py/${w.wheel}`, location.href).href,
      urdfUrl: new URL(sd.urdf, location.href).href,
      urdfRel: sd.urdf.replace(/^robots\//, ""),
      config: sd.config,
      side: hand.side,
    }));
  }

  function onWorker(e) {
    const msg = e.data;
    if (msg.type === "ready" && msg.id === config) {
      ready = true;
      say(`Show your ${hand.side} hand to the camera.`);
    } else if (msg.type === "error") {
      say(`The retargeting engine failed to start: ${msg.message}`);
    } else if (msg.type === "pose") {
      busy = false;
      if (msg.id !== config || !msg.values) return;
      solveMs = msg.ms;
      const n = hand.sideData.joints.length;
      const q = Float32Array.from(msg.values.slice(0, n));
      const kp = msg.values.slice(n);
      const r = hand.sideData.palm;
      const p = new Float32Array(63);
      for (let k = 0; k < 21; k++) {
        for (let i = 0; i < 3; i++) {
          p[k * 3 + i] = r[i][0] * kp[k * 3] + r[i][1] * kp[k * 3 + 1] + r[i][2] * kp[k * 3 + 2];
        }
      }
      onPose(q, p);
    }
  }

  function pickHand(res) {
    const want = hand.side === "right" ? "Left" : "Right";
    const idx = res.handedness.findIndex((h) => h[0] && h[0].categoryName === want);
    return idx >= 0 ? idx : res.worldLandmarks.length === 1 ? 0 : -1;
  }

  function loop() {
    if (!running) return;
    if (landmarker && video.readyState >= 2 && video.currentTime !== lastTime) {
      lastTime = video.currentTime;
      const res = landmarker.detectForVideo(video, performance.now());
      const idx = pickHand(res);
      if (idx < 0) {
        if (ready) say(`No ${hand.side} hand in view.`);
      } else if (ready && !busy) {
        busy = true;
        worker.postMessage({ type: "solve", id: config, xr: landmarksToXr(res.worldLandmarks[idx], hand.side) });
        say(`Tracking. Retargeting takes ${solveMs.toFixed(1)} ms per frame.`);
      }
    }
    requestAnimationFrame(loop);
  }

  function start() {
    if (!hand || !hand.sideData.config) {
      say("This hand has no retargeting config, so live mode is not available for it.");
      return;
    }
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      say("This browser does not expose a camera here. Open the page over HTTPS in a current browser.");
      return;
    }
    button.disabled = true;
    say("Waiting for camera permission.");
    wheel.then((w) => w || say("This build has no retargeting engine for live mode."));
    navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 480, facingMode: "user" }, audio: false })
      .then((s) => {
        stream = s;
        video.srcObject = s;
        return video.play();
      })
      .then(() => {
        running = true;
        button.textContent = "Stop camera";
        button.disabled = false;
        say("Loading the hand tracker and the retargeting engine. The first load downloads about 40 MB.");
        if (!worker) {
          worker = new Worker(new URL("retarget-worker.js", import.meta.url), { type: "module" });
          worker.onmessage = onWorker;
          configure();
        }
        requestAnimationFrame(loop);
        return landmarker || import(`${TASKS}/vision_bundle.mjs`).then(({ FilesetResolver, HandLandmarker }) =>
          FilesetResolver.forVisionTasks(`${TASKS}/wasm`).then((files) => HandLandmarker.createFromOptions(files, {
            baseOptions: { modelAssetPath: MODEL, delegate: "GPU" },
            runningMode: "VIDEO",
            numHands: 2,
          })));
      })
      .then((lm) => {
        landmarker = lm || landmarker;
        if (ready) say(`Show your ${hand.side} hand to the camera.`);
      })
      .catch((err) => {
        button.disabled = false;
        stop();
        const name = err && err.name;
        if (name === "NotAllowedError") say("Camera access was blocked. Allow camera access for this page, then press Start camera.");
        else if (name === "NotFoundError" || name === "OverconstrainedError") say("No camera was found. Connect a camera, then press Start camera.");
        else say(`Live mode could not start: ${err && err.message ? err.message : err}`);
      });
  }

  function stop() {
    running = false;
    if (stream) stream.getTracks().forEach((t) => t.stop());
    stream = null;
    video.srcObject = null;
    button.textContent = "Start camera";
  }

  button.addEventListener("click", () => (running ? stop() : start()));

  return {
    setHand(current) {
      hand = current;
      configure();
    },
    stop() {
      stop();
      say("");
    },
  };
}
