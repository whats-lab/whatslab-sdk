import { loadPyodide } from "https://cdn.jsdelivr.net/pyodide/v0.29.3/full/pyodide.mjs";

const SETUP = `
import os
import time
import numpy as np
os.environ["WHATSLAB_MODELS_ROOT"] = "/models"
from whatslab.retarget.dex import DexEngine

engine = None
index = None


def configure(config, side):
    global engine, index
    engine = DexEngine(config, side=side)
    kin = engine.kinematics
    actuated = list(kin.actuated_joint_names)
    names = actuated + [n for n in kin.joint_names if n not in actuated]
    index = [kin.joint_names.index(n) for n in names]


def solve(buffer):
    t0 = time.perf_counter()
    q = engine.solve(np.asarray(buffer.to_py(), dtype=float).reshape(26, 8))
    if q is None:
        return None
    return np.concatenate([[1000 * (time.perf_counter() - t0)], engine.full_qpos(q)[index]]).tolist()
`;

let py = null;
let configure = null;
let solve = null;

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

async function boot(wheelUrl) {
  stage("pyodide");
  py = await loadPyodide();
  stage("packages");
  await py.loadPackage(["numpy", "scipy", "pyyaml", "micropip"]);
  stage("wheel");
  const micropip = py.pyimport("micropip");
  await micropip.install(wheelUrl, { deps: false });
  py.runPython(SETUP);
  configure = py.globals.get("configure");
  solve = py.globals.get("solve");
}

let booting = null;

self.onmessage = (e) => {
  const msg = e.data;
  if (msg.type === "init") {
    booting = booting || boot(msg.wheelUrl);
    booting
      .then(() => fetch(msg.urdfUrl))
      .then((r) => (r.ok ? r.text() : Promise.reject(new Error(`URDF ${r.status}`))))
      .then((text) => {
        const path = `/models/${msg.urdfRel}`;
        py.FS.mkdirTree(path.slice(0, path.lastIndexOf("/")));
        py.FS.writeFile(path, text);
        stage("engine");
        configure(msg.config, msg.side);
        self.postMessage({ type: "ready", id: msg.id });
      })
      .catch((err) => self.postMessage({ type: "error", message: String(err && err.message ? err.message : err) }));
  } else if (msg.type === "solve" && solve) {
    const t0 = performance.now();
    const res = solve(msg.xr);
    const out = res ? res.toJs() : null;
    if (res) res.destroy();
    self.postMessage({ type: "pose", id: msg.id, values: out, ms: performance.now() - t0 });
  }
};
