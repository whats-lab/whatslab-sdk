import { loadPyodide } from "https://cdn.jsdelivr.net/pyodide/v0.29.3/full/pyodide.mjs";

const SETUP = `
import os
import numpy as np
from scipy.spatial.transform import Rotation
os.environ["WHATSLAB_MODELS_ROOT"] = "/models"
from whatslab.core.constants import OPENXR_TO_HAND21_AXES, XR_WRIST
from whatslab.retarget.dex import DexEngine
from whatslab.retarget.dex.preprocess import mano_palm_frame, mirror_xr_joints, xr_to_mano

engine = None
index = None


def configure(config, side):
    global engine, index
    engine = DexEngine(config, side=side)
    kin = engine.kinematics
    index = [kin.joint_names.index(n) for n in kin.actuated_joint_names]


def human(xr):
    a = mirror_xr_joints(xr) if engine.mirror else xr
    rot = Rotation.from_quat(a[:, :4]).as_matrix()
    frame = mano_palm_frame(xr_to_mano(a), engine.config.side)
    if frame is None:
        return None
    to_canon = frame.T @ OPENXR_TO_HAND21_AXES @ rot[XR_WRIST].T
    quat = Rotation.from_matrix(to_canon @ rot).as_quat()
    return np.concatenate([quat, (a[:, 4:7] - a[XR_WRIST, 4:7]) @ to_canon.T], axis=1).ravel()


def solve(buffer):
    xr = np.asarray(buffer.to_py(), dtype=float).reshape(26, 8)
    q = engine.solve(xr)
    pose = human(xr)
    if q is None or pose is None:
        return None
    return np.concatenate([engine.full_qpos(q)[index], pose]).tolist()
`;

let py = null;
let configure = null;
let solve = null;

const stage = (name) => self.postMessage({ type: "stage", stage: name, t: performance.now() });

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
