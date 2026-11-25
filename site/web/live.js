const TASKS = "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@1.1.1";
const MODEL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task";
const RAW_BYTES = { "vision_wasm_internal.wasm": 13519922, "hand_landmarker.task": 7819105 };
const PARENTS = [-1, 0, 1, 2, 3, 0, 5, 6, 7, 8, 0, 10, 11, 12, 13, 0, 15, 16, 17, 18, 0, 20, 21, 22, 23];

function fetchBytes(url, onBytes) {
  return fetch(url).then((r) => {
    if (!r.ok) return { error: `HTTP ${r.status} from ${new URL(url).host}` };
    const total = RAW_BYTES[new URL(url).pathname.split("/").pop()] || Number(r.headers.get("content-length")) || 0;
    const reader = r.body.getReader();
    const chunks = [];
    let got = 0;
    onBytes(0, total);
    const pump = () => reader.read().then(({ done, value }) => {
      if (done) {
        const data = new Uint8Array(got);
        chunks.reduce((o, c) => (data.set(c, o), o + c.length), 0);
        return { data };
      }
      chunks.push(value);
      got += value.length;
      onBytes(got, total);
      return pump();
    });
    return pump();
  }).then((res) => res, (err) => ({ error: `${new URL(url).host}: ${err && err.message ? err.message : err}` }));
}

function showRow(root, part, { frac, text, failed }) {
  const row = root.querySelector(`[data-part="${part}"]`);
  const pct = Math.round(Math.min(Math.max(frac, 0), 1) * 100);
  row.classList.toggle("failed", Boolean(failed));
  row.setAttribute("aria-valuenow", String(pct));
  row.querySelector(".loading-fill").style.width = `${pct}%`;
  row.querySelector(".loading-value").textContent = text;
}

const mb = (n) => (n / 1e6).toFixed(1);

export function createLive({ video, overlay, note, button, onPose, bars }) {
  let landmarker = null;
  let worker = null;
  let stream = null;
  let hand = null;
  let ready = false;
  let busy = false;
  let lastTime = -1;
  let running = false;
  let solveMs = 0;
  let fitMs = 0;
  let tracked = false;
  let sent = null;
  let config = 0;
  let tracker = "waiting";
  let engine = "waiting";
  const STAGES = {
    pyodide: ["Pyodide", 0, 0.4],
    packages: ["numpy, scipy", 0.4, 0.88],
    wheel: ["whatslab-sdk", 0.88, 0.95],
    engine: ["building engine", 0.95, 1],
  };
  const parts = { wasm: [0, 0], model: [0, 0] };
  let trackerFailed = false;
  let engineFailed = false;

  function renderTracker(label) {
    const got = parts.wasm[0] + parts.model[0];
    const total = parts.wasm[1] + parts.model[1];
    const frac = total ? Math.min(got / total, 0.99) : 0;
    showRow(bars, "tracker", { frac, text: total ? `${Math.round(frac * 100)}% · ${mb(got)} / ${mb(total)} MB` : label });
  }

  function renderEngine(stageName, got, total) {
    const [label, lo, hi] = STAGES[stageName];
    const inner = total ? Math.min(got / total, 1) : 0;
    const frac = lo + (hi - lo) * inner;
    showRow(bars, "engine", {
      frac,
      text: `${Math.round(frac * 100)}% · ${label}${total ? ` ${mb(got)} / ${mb(total)} MB` : ""}`,
    });
  }

  function settle() {
    const done = (landmarker || trackerFailed) && (ready || engineFailed);
    bars.hidden = !running || (done && !trackerFailed && !engineFailed);
  }

  const wheel = fetch("py/wheel.json").then((r) => (r.ok ? r.json() : null)).catch(() => null);

  const say = (text) => { note.textContent = text; };
  const progress = () => say(`Hand tracker: ${tracker}. Retargeting engine: ${engine}.`);

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
      urdfUrl: new URL(sd.engineUrdf, location.href).href,
      urdfRel: sd.engineUrdf.replace(/^robots\//, ""),
      fitUrl: new URL("handfit.py", location.href).href,
      glbUrl: new URL(`assets/generic-hand/${hand.side}.glb`, location.href).href,
      config: sd.config,
      side: hand.side,
    }));
  }

  function onWorker(e) {
    const msg = e.data;
    if (msg.type === "stage" || msg.type === "progress") {
      renderEngine(msg.stage, msg.received || 0, msg.expected || 0);
      if (engine !== "loading") {
        engine = "loading";
        progress();
      }
    } else if (msg.type === "ready" && msg.id === config) {
      ready = true;
      engine = "ready";
      showRow(bars, "engine", { frac: 1, text: "ready" });
      settle();
      if (landmarker) say(`Show your ${hand.side} hand to the camera.`);
      else progress();
    } else if (msg.type === "error") {
      engine = "failed";
      engineFailed = true;
      showRow(bars, "engine", { frac: 0, text: "failed", failed: true });
      settle();
      say(`The retargeting engine failed to start: ${msg.message}`);
    } else if (msg.type === "pose") {
      busy = false;
      if (msg.id !== config || !msg.values) return;
      solveMs = msg.ms;
      fitMs = msg.values[0];
      const n = hand.sideData.joints.length;
      const v = msg.values;
      onPose(Float32Array.from(v.slice(1, 1 + n)), Float32Array.from(v.slice(1 + n, 1 + n + 182)));
      drawFit(v.slice(1 + n + 182));
    }
  }

  function drawFit(uv) {
    const ctx = overlay.getContext("2d");
    overlay.width = video.videoWidth || 640;
    overlay.height = video.videoHeight || 480;
    ctx.clearRect(0, 0, overlay.width, overlay.height);
    if (!uv || !uv.length) return;
    const color = getComputedStyle(overlay).color;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2;
    for (let j = 1; j < PARENTS.length; j++) {
      const p = PARENTS[j];
      ctx.beginPath();
      ctx.moveTo(uv[p * 2] * overlay.width, uv[p * 2 + 1] * overlay.height);
      ctx.lineTo(uv[j * 2] * overlay.width, uv[j * 2 + 1] * overlay.height);
      ctx.stroke();
    }
    for (let j = 0; j < PARENTS.length; j++) {
      ctx.beginPath();
      ctx.arc(uv[j * 2] * overlay.width, uv[j * 2 + 1] * overlay.height, 3, 0, 2 * Math.PI);
      ctx.fill();
    }
    if (!sent) return;
    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#15171b";
    ctx.lineWidth = 1;
    for (let k = 0; k < 21; k++) {
      ctx.beginPath();
      ctx.arc(sent[k * 2] * overlay.width, sent[k * 2 + 1] * overlay.height, 2.5, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
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
        if (tracked && worker) worker.postMessage({ type: "reset" });
        tracked = false;
        drawFit(null);
      } else if (ready && !busy) {
        busy = true;
        tracked = true;
        worker.postMessage({
          type: "solve",
          id: config,
          world: Float64Array.from(res.worldLandmarks[idx].flatMap((p) => [p.x, p.y, p.z])),
          image: (sent = Float64Array.from(res.landmarks[idx].flatMap((p) => [p.x, p.y]))),
          aspect: video.videoWidth / video.videoHeight,
        });
        say(`Tracking. Hand model fit ${fitMs.toFixed(1)} ms, fit and retargeting ${solveMs.toFixed(1)} ms per frame.`);
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
    wheel.then((w) => {
      if (!w) engine = "not included in this build";
    });
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
        tracker = landmarker ? "ready" : "loading";
        if (!landmarker) renderTracker("loading library");
        if (!ready && !engineFailed) showRow(bars, "engine", { frac: 0, text: "starting" });
        settle();
        progress();
        if (!worker) {
          worker = new Worker(new URL("retarget-worker.js", import.meta.url), { type: "module" });
          worker.onmessage = onWorker;
          worker.onerror = (e) => {
            engine = "failed";
            engineFailed = true;
            showRow(bars, "engine", { frac: 0, text: "failed", failed: true });
            settle();
            say(`The retargeting engine could not start: ${e.message || "its script or Pyodide could not be loaded"}.`);
          };
          configure();
        }
        requestAnimationFrame(loop);
        if (!landmarker) loadTracker().then(onTracker);
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

  function loadTracker() {
    return import(`${TASKS}/vision_bundle.mjs`).then(({ FilesetResolver, HandLandmarker }) => FilesetResolver
      .forVisionTasks(`${TASKS}/wasm`)
      .then((files) => Promise.all([
        fetchBytes(files.wasmBinaryPath, (got, total) => {
          parts.wasm = [got, total];
          renderTracker("downloading");
        }),
        fetchBytes(MODEL, (got, total) => {
          parts.model = [got, total];
          renderTracker("downloading");
        }),
      ]).then(([wasm, model]) => {
        if (wasm.error || model.error) return { error: wasm.error || model.error };
        showRow(bars, "tracker", { frac: 0.99, text: "starting" });
        const wasmUrl = URL.createObjectURL(new Blob([wasm.data], { type: "application/wasm" }));
        return HandLandmarker.createFromOptions({ ...files, wasmBinaryPath: wasmUrl }, {
          baseOptions: { modelAssetBuffer: model.data, delegate: "GPU" },
          runningMode: "VIDEO",
          numHands: 2,
        }).then((lm) => ({ lm }));
      })))
      .then((res) => res, (err) => ({ error: err && err.message ? err.message : String(err) }));
  }

  function onTracker(res) {
    if (res.error) {
      tracker = "failed";
      trackerFailed = true;
      showRow(bars, "tracker", { frac: 0, text: "failed", failed: true });
      settle();
      say(`The hand tracker could not be loaded: ${res.error}`);
      return;
    }
    landmarker = res.lm;
    tracker = "ready";
    showRow(bars, "tracker", { frac: 1, text: "ready" });
    settle();
    if (ready) say(`Show your ${hand.side} hand to the camera.`);
    else if (!engineFailed) progress();
  }

  function stop() {
    running = false;
    bars.hidden = true;
    if (stream) stream.getTracks().forEach((t) => t.stop());
    stream = null;
    video.srcObject = null;
    drawFit(null);
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
