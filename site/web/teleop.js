import * as THREE from "three";

export const PARENT = [-1, 0, 1, 2, 3, 0, 5, 6, 7, 8, 0, 10, 11, 12, 13, 0, 15, 16, 17, 18, 0, 20, 21, 22, 23];
export const TIPS = [4, 9, 14, 19, 24];
const CHAINS = [[1, 2, 3], [6, 7, 8], [11, 12, 13], [16, 17, 18], [21, 22, 23]];
const LIMITS = {
  thumb: [[-0.6, 1.3], [-0.6, 1.0], [-0.3, 1.4]],
  finger: [[-0.35, 1.6], [-0.35, 0.35], [-0.1, 1.9]],
};
const COUPLE = { thumb: 0.8, finger: 0.7 };
const X = new THREE.Vector3(1, 0, 0);
const Y = new THREE.Vector3(0, 1, 0);
const A = new THREE.Matrix3().set(0, -1, 0, -1, 0, 0, 0, 0, -1);
const MIRROR = new THREE.Matrix3().set(-1, 0, 0, 0, 1, 0, 0, 0, 1);

function palmFrame(w, i, m, p, side) {
  const z = m.clone().sub(w).normalize();
  const y = (side === "right" ? i.clone().sub(p) : p.clone().sub(i));
  y.addScaledVector(z, -y.dot(z)).normalize();
  const x = new THREE.Vector3().crossVectors(y, z);
  return new THREE.Matrix3().set(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z);
}

function quatFromMatrix3(m) {
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().setFromMatrix3(m));
}

export class HandModel {
  constructor(bones, side) {
    this.side = side;
    const pos = bones.map((b) => b.position.clone());
    const rot = bones.map((b) => b.quaternion.clone());
    this.rel = rot.map((q, j) => (j ? rot[PARENT[j]].clone().invert().multiply(q) : q.clone()));
    this.off = pos.map((p, j) => (j ? p.clone().sub(pos[PARENT[j]]).applyQuaternion(rot[PARENT[j]].clone().invert()) : p.clone()));
    this.angles = bones.map(() => ({ flex: 0, abd: 0 }));
    this.fk();
    const rw = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(this.rot[0]));
    const local = (j) => this.pos[j].clone().sub(this.pos[0]).applyMatrix3(rw.clone().transpose()).applyMatrix3(A);
    const frame = palmFrame(local(0), local(6), local(11), local(21), side);
    this.toCanon = frame.clone().transpose().multiply(A).multiply(rw.clone().transpose());
    this.toCanonQ = quatFromMatrix3(this.toCanon);
  }

  fk() {
    const n = this.rel.length;
    this.pos = new Array(n);
    this.rot = new Array(n);
    this.pos[0] = this.off[0].clone();
    this.rot[0] = this.rel[0].clone();
    const qa = new THREE.Quaternion();
    const qf = new THREE.Quaternion();
    for (let j = 1; j < n; j++) {
      const p = PARENT[j];
      const a = this.angles[j];
      qa.setFromAxisAngle(Y, a.abd);
      qf.setFromAxisAngle(X, -a.flex);
      this.rot[j] = this.rot[p].clone().multiply(this.rel[j]).multiply(qa).multiply(qf);
      this.pos[j] = this.pos[p].clone().add(this.off[j].clone().applyQuaternion(this.rot[p]));
    }
  }

  canonical() {
    const out = new Float32Array(26 * 7);
    const q = new THREE.Quaternion();
    const v = new THREE.Vector3();
    for (let k = 0; k < 26; k++) {
      const j = Math.max(k - 1, 0);
      q.copy(this.toCanonQ).multiply(this.rot[j]);
      if (k === 0) v.copy(this.pos[0]).add(this.pos[11]).multiplyScalar(0.5);
      else v.copy(this.pos[j]);
      v.sub(this.pos[0]).applyMatrix3(this.toCanon);
      out.set([q.x, q.y, q.z, q.w, v.x, v.y, v.z], k * 7);
    }
    return out;
  }

  xr(mirror) {
    const out = new Float64Array(26 * 8);
    const rw = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(this.rot[0]));
    const rwt = rw.clone().transpose();
    for (let k = 0; k < 26; k++) {
      const j = Math.max(k - 1, 0);
      const p = k === 0 ? this.pos[0].clone().add(this.pos[11]).multiplyScalar(0.5) : this.pos[j].clone();
      let r = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(this.rot[j]));
      p.sub(this.pos[0]).applyMatrix3(rwt);
      r = rwt.clone().multiply(r);
      if (mirror) {
        p.applyMatrix3(MIRROR);
        r = MIRROR.clone().multiply(r).multiply(MIRROR);
      }
      const q = quatFromMatrix3(r);
      out.set([q.x, q.y, q.z, q.w, p.x, p.y, p.z, 0.01], k * 8);
    }
    return out;
  }

  chainValues(f) {
    const [a, b, c] = CHAINS[f];
    return [this.angles[a].flex, this.angles[a].abd, this.angles[b].flex, this.angles[c].flex];
  }

  setChain(f, x) {
    const kind = f === 0 ? "thumb" : "finger";
    const lim = LIMITS[kind];
    const v = x.map((t, i) => Math.min(Math.max(t, lim[i][0]), lim[i][1]));
    const [a, b, c] = CHAINS[f];
    this.angles[a].flex = v[0];
    this.angles[a].abd = v[1];
    this.angles[b].flex = v[2];
    this.angles[c].flex = v[2] * COUPLE[kind];
    return v;
  }

  solveTip(f, target, iters = 12) {
    let x = this.chainValues(f).slice(0, 3);
    const tip = TIPS[f];
    const err = () => target.clone().sub(this.pos[tip]);
    x = this.setChain(f, x);
    this.fk();
    for (let it = 0; it < iters; it++) {
      const e = err();
      if (e.length() < 2e-4) break;
      const J = [];
      for (let i = 0; i < 3; i++) {
        const d = x.slice();
        d[i] += 1e-3;
        this.setChain(f, d);
        this.fk();
        J.push(this.pos[tip].clone().sub(target).add(e).multiplyScalar(1e3));
        this.setChain(f, x);
      }
      this.fk();
      const jt = new THREE.Matrix3().set(J[0].x, J[0].y, J[0].z, J[1].x, J[1].y, J[1].z, J[2].x, J[2].y, J[2].z);
      const jtj = new THREE.Matrix3().multiplyMatrices(jt, jt.clone().transpose());
      const lam = 1e-4;
      const el = jtj.elements;
      el[0] += lam;
      el[4] += lam;
      el[8] += lam;
      const g = e.clone().applyMatrix3(jt);
      const step = g.applyMatrix3(jtj.clone().invert());
      x = this.setChain(f, [x[0] + step.x, x[1] + step.y, x[2] + step.z]);
      this.fk();
    }
    return x;
  }

  reachTip(f, target) {
    let best = null;
    for (const seed of [this.chainValues(f).slice(0, 3), [0.4, 0.2, 0.4], [0.9, 0.3, 0.8], [0.2, 0.6, 0.2], [1.2, 0.0, 0.3]]) {
      this.setChain(f, seed);
      this.fk();
      const x = this.solveTip(f, target, 40);
      const err = this.pos[TIPS[f]].distanceTo(target);
      if (!best || err < best.err) best = { x, err };
    }
    this.setChain(f, best.x);
    this.fk();
    return best.err;
  }

  fromCanonical(p) {
    const rotAt = (j) => new THREE.Quaternion(p[(j + 1) * 7], p[(j + 1) * 7 + 1], p[(j + 1) * 7 + 2], p[(j + 1) * 7 + 3]).normalize();
    const fwd = new THREE.Vector3();
    for (const chain of CHAINS) {
      for (const j of chain) {
        const d = this.rel[j].clone().invert().multiply(rotAt(PARENT[j]).invert()).multiply(rotAt(j));
        fwd.set(0, 0, -1).applyQuaternion(d);
        this.angles[j].flex = Math.asin(Math.min(Math.max(-fwd.y, -1), 1));
        this.angles[j].abd = j === chain[0] ? Math.atan2(-fwd.x, -fwd.z) : 0;
      }
    }
    CHAINS.forEach((_, f) => this.setChain(f, this.chainValues(f).slice(0, 3)));
    this.fk();
  }

  reset() {
    this.angles.forEach((a) => { a.flex = 0; a.abd = 0; });
    this.fk();
  }

  preset(name) {
    this.reset();
    const curl = (f, mcp, pip) => this.setChain(f, [mcp, 0, pip]);
    if (name === "fist") {
      [1, 2, 3, 4].forEach((f) => curl(f, 1.45, 1.7));
      this.fk();
      this.reachTip(0, this.pos[12].clone().lerp(this.pos[7], 0.5));
    } else if (name === "pinch") {
      curl(1, 0.75, 0.75);
      [2, 3, 4].forEach((f) => curl(f, 0.15, 0.2));
      this.fk();
      for (let k = 0; k < 3; k++) {
        this.reachTip(0, this.pos[9].clone());
        this.reachTip(1, this.pos[9].clone().lerp(this.pos[4], 0.5));
      }
      this.reachTip(0, this.pos[9].clone());
    } else if (name === "v") {
      [3, 4].forEach((f) => curl(f, 1.45, 1.7));
      this.setChain(1, [0, this.side === "right" ? 0.18 : -0.18, 0]);
      this.setChain(2, [0, this.side === "right" ? -0.12 : 0.12, 0]);
      this.fk();
      this.reachTip(0, this.pos[17].clone().lerp(this.pos[22], 0.5));
    }
    this.fk();
  }
}
