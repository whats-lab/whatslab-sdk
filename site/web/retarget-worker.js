import { loadPyodide } from "https://cdn.jsdelivr.net/pyodide/v0.29.3/full/pyodide.mjs";

const SETUP = `
import os
import numpy as np
os.environ["WHATSLAB_MODELS_ROOT"] = "/models"
from whatslab.retarget.dex import DexEngine

engine = None
index = None


def configure(config, side):
    global engine, index
    engine = DexEngine(config, side=side)
    kin = engine.kinematics
    index = [kin.joint_names.index(n) for n in kin.actuated_joint_names]


def solve(buffer):
    xr = np.asarray(buffer.to_py(), dtype=float).reshape(26, 8)
    q = engine.solve(xr)
    if q is None:
        return None
    return np.concatenate([engine.full_qpos(q)[index], engine.human_keypoints(xr).ravel()]).tolist()
`;

let py = null;
let configure = null;
let solve = null;

async function boot(wheelUrl) {
  py = await loadPyodide();
  await py.loadPackage(["numpy", "scipy", "pyyaml", "micropip"]);
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
