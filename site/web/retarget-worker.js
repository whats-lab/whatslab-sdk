import { loadPyodide } from "https://cdn.jsdelivr.net/pyodide/v0.29.3/full/pyodide.mjs";

const SETUP = `
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation
os.environ["WHATSLAB_MODELS_ROOT"] = "/models"
from whatslab.core.constants import OPENXR_TO_HAND21_AXES, XR_WRIST
from whatslab.retarget.dex import DexEngine
from whatslab.retarget.dex.preprocess import mano_palm_frame, mirror_xr_joints, xr_to_mano
from handfit import HandFit

engine = None
index = None
fit = None


def configure(config, side, glb):
    global engine, index, fit
    engine = DexEngine(config, side=side)
    kin = engine.kinematics
    actuated = list(kin.actuated_joint_names)
    names = actuated + [n for n in kin.joint_names if n not in actuated]
    index = [kin.joint_names.index(n) for n in names]
    fit = HandFit(bytes(glb.to_py()), side)


def reset():
    fit.x = None


def human(xr):
    a = mirror_xr_joints(xr) if engine.mirror else xr
    rot = Rotation.from_quat(a[:, :4]).as_matrix()
    frame = mano_palm_frame(xr_to_mano(a), engine.config.side)
    if frame is None:
        return None
    to_canon = frame.T @ OPENXR_TO_HAND21_AXES @ rot[XR_WRIST].T
    quat = Rotation.from_matrix(to_canon @ rot).as_quat()
    return np.concatenate([quat, (a[:, 4:7] - a[XR_WRIST, 4:7]) @ to_canon.T], axis=1).ravel()


def solve(world, image, aspect):
    t0 = time.perf_counter()
    res = fit.fit(np.asarray(world.to_py(), dtype=float), np.asarray(image.to_py(), dtype=float), aspect)
    if res is None:
        return None
    xr = res[0]
    fit_ms = 1000 * (time.perf_counter() - t0)
    q = engine.solve(xr)
    pose = human(xr)
    if q is None or pose is None:
        return None
    return np.concatenate([[fit_ms], engine.full_qpos(q)[index], pose, fit.uv.ravel()]).tolist()
`;

let py = null;
let configure = null;
let solve = null;
let reset = null;

const RAW_BYTES = {
  "pyodide-lock.json": 120010,
  "python_stdlib.zip": 2423989,
  "pyodide.asm.wasm": 8647684,
  "numpy-2.2.5-cp313-cp313-pyodide_2025_0_wasm32.whl": 3103966,
  "scipy-1.14.1-cp313-cp313-pyodide_2025_0_wasm32.whl": 13258840,
  "libopenblas-0.3.26.zip": 2119540,
  "pyyaml-6.0.2-cp313-cp313-pyodide_2025_0_wasm32.whl": 117402,
  "micropip-0.11.0-py3-none-any.whl": 115409,
};
const STAGE_BYTES = { pyodide: 11191683, packages: 18715157 };
let current = "pyodide";
let seen = 0;
let received = 0;
let expected = 0;
let reported = 0;

const report = (force) => {
  const now = performance.now();
  if (!force && now - reported < 100) return;
  reported = now;
  expected = Math.max(STAGE_BYTES[current] || 0, seen);
  self.postMessage({ type: "progress", stage: current, received: Math.min(received, expected), expected });
};

const stage = (name) => {
  current = name;
  received = 0;
  expected = 0;
  seen = 0;
  self.postMessage({ type: "stage", stage: name, t: performance.now() });
};

const nativeFetch = self.fetch.bind(self);
self.fetch = (input, init) => nativeFetch(input, init).then((r) => {
  if (!r.ok || !r.body) return r;
  const name = new URL(r.url).pathname.split("/").pop();
  seen += RAW_BYTES[name] || Number(r.headers.get("content-length")) || 0;
  report(true);
  const reader = r.body.getReader();
  const body = new ReadableStream({
    pull: (ctrl) => reader.read().then(({ done, value }) => {
      if (done) {
        report(true);
        ctrl.close();
        return;
      }
      received += value.byteLength;
      report(false);
      ctrl.enqueue(value);
    }),
  });
  return new Response(body, { status: r.status, statusText: r.statusText, headers: r.headers });
});

self.addEventListener("unhandledrejection", (e) => {
  self.postMessage({ type: "error", message: String(e.reason && e.reason.message ? e.reason.message : e.reason) });
});

async function boot(wheelUrl, fitUrl) {
  stage("pyodide");
  py = await loadPyodide();
  stage("packages");
  await py.loadPackage(["numpy", "scipy", "pyyaml", "micropip"]);
  stage("wheel");
  const micropip = py.pyimport("micropip");
  await micropip.install(wheelUrl, { deps: false });
  const fitSource = await fetch(fitUrl).then((r) => (r.ok ? r.text() : Promise.reject(new Error(`handfit.py ${r.status}`))));
  py.FS.writeFile("/home/pyodide/handfit.py", fitSource);
  py.runPython(SETUP);
  reset = py.globals.get("reset");
  configure = py.globals.get("configure");
  solve = py.globals.get("solve");
}

let booting = null;

self.onmessage = (e) => {
  const msg = e.data;
  if (msg.type === "init") {
    booting = booting || boot(msg.wheelUrl, msg.fitUrl);
    booting
      .then(() => Promise.all([fetch(msg.urdfUrl), fetch(msg.glbUrl)]))
      .then(([r, g]) => (r.ok && g.ok
        ? Promise.all([r.text(), g.arrayBuffer()])
        : Promise.reject(new Error(`URDF ${r.status}, hand model ${g.status}`))))
      .then(([text, glb]) => {
        const path = `/models/${msg.urdfRel}`;
        py.FS.mkdirTree(path.slice(0, path.lastIndexOf("/")));
        py.FS.writeFile(path, text);
        stage("engine");
        configure(msg.config, msg.side, new Uint8Array(glb));
        self.postMessage({ type: "ready", id: msg.id });
      })
      .catch((err) => self.postMessage({ type: "error", message: String(err && err.message ? err.message : err) }));
  } else if (msg.type === "reset" && reset) {
    reset();
  } else if (msg.type === "solve" && solve) {
    const t0 = performance.now();
    const res = solve(msg.world, msg.image, msg.aspect);
    const out = res ? res.toJs() : null;
    if (res) res.destroy();
    self.postMessage({ type: "pose", id: msg.id, values: out, ms: performance.now() - t0 });
  }
};
