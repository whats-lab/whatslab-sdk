import * as THREE from "three";

const COMPONENT = { 5120: Int8Array, 5121: Uint8Array, 5122: Int16Array, 5123: Uint16Array, 5125: Uint32Array, 5126: Float32Array };
const WIDTH = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4, MAT4: 16 };
const FINGERS = [
  ["Thumb", 2, ["Metacarpal", "Proximal", "Distal"]],
  ["Index", 6, ["Metacarpal", "Proximal", "Intermediate", "Distal"]],
  ["Middle", 11, ["Metacarpal", "Proximal", "Intermediate", "Distal"]],
  ["Ring", 16, ["Metacarpal", "Proximal", "Intermediate", "Distal"]],
  ["Little", 21, ["Metacarpal", "Proximal", "Intermediate", "Distal"]],
];
export const DRIVEN_BONES = { XRHand_Wrist: 1 };
for (const [finger, base, segments] of FINGERS) {
  segments.forEach((segment, k) => { DRIVEN_BONES[`XRHand_${finger}${segment}`] = base + k; });
}

function readGlb(buffer) {
  const view = new DataView(buffer);
  let gltf = null;
  let binary = null;
  for (let off = 12; off < buffer.byteLength;) {
    const length = view.getUint32(off, true);
    const kind = view.getUint32(off + 4, true);
    if (kind === 0x4e4f534a) gltf = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, off + 8, length)));
    else if (kind === 0x004e4942) binary = buffer.slice(off + 8, off + 8 + length);
    off += 8 + length;
  }
  return { gltf, binary };
}

function accessor(gltf, binary, index) {
  const acc = gltf.accessors[index];
  const bv = gltf.bufferViews[acc.bufferView];
  const Type = COMPONENT[acc.componentType];
  const width = WIDTH[acc.type];
  const offset = (bv.byteOffset || 0) + (acc.byteOffset || 0);
  const stride = bv.byteStride || Type.BYTES_PER_ELEMENT * width;
  const out = new Float64Array(acc.count * width);
  const dv = new DataView(binary);
  const get = {
    5120: (o) => dv.getInt8(o), 5121: (o) => dv.getUint8(o), 5122: (o) => dv.getInt16(o, true),
    5123: (o) => dv.getUint16(o, true), 5125: (o) => dv.getUint32(o, true), 5126: (o) => dv.getFloat32(o, true),
  }[acc.componentType];
  const max = { 5121: 255, 5123: 65535, 5125: 4294967295, 5120: 127, 5122: 32767 }[acc.componentType];
  for (let i = 0; i < acc.count; i++) {
    for (let c = 0; c < width; c++) {
      const v = get(offset + i * stride + c * Type.BYTES_PER_ELEMENT);
      out[i * width + c] = acc.normalized && max ? v / max : v;
    }
  }
  return { data: out, width, count: acc.count };
}

function local(node) {
  if (node.matrix) return new THREE.Matrix4().fromArray(node.matrix);
  const q = new THREE.Quaternion().fromArray(node.rotation || [0, 0, 0, 1]);
  const s = new THREE.Vector3().fromArray(node.scale || [1, 1, 1]);
  const t = new THREE.Vector3().fromArray(node.translation || [0, 0, 0]);
  return new THREE.Matrix4().compose(t, q, s);
}

function rigid(m) {
  let r = new THREE.Matrix3().setFromMatrix4(m);
  for (let k = 0; k < 20; k++) {
    const inv = r.clone().invert().transpose();
    const e = r.elements;
    const f = inv.elements;
    for (let i = 0; i < 9; i++) e[i] = 0.5 * (e[i] + f[i]);
  }
  return { R: r, p: new THREE.Vector3().setFromMatrixPosition(m) };
}

export function parseMetaHand(buffer) {
  const { gltf, binary } = readGlb(buffer);
  const nodes = gltf.nodes;
  const parent = nodes.map(() => -1);
  nodes.forEach((n, i) => (n.children || []).forEach((c) => { parent[c] = i; }));
  const world = new Map();
  const w = (i) => {
    if (!world.has(i)) world.set(i, parent[i] < 0 ? local(nodes[i]) : w(parent[i]).clone().multiply(local(nodes[i])));
    return world.get(i);
  };
  const meshNode = nodes.findIndex((n) => n.mesh !== undefined && n.skin !== undefined);
  const skin = gltf.skins[nodes[meshNode].skin];
  const joints = skin.joints;
  const slot = new Map(joints.map((j, k) => [j, k]));
  const prim = gltf.meshes[nodes[meshNode].mesh].primitives[0];
  const ibm0 = new THREE.Matrix4().fromArray(accessor(gltf, binary, skin.inverseBindMatrices).data.slice(0, 16));
  const skinMat = w(joints[0]).clone().multiply(ibm0);
  const pos = accessor(gltf, binary, prim.attributes.POSITION);
  const nrm = accessor(gltf, binary, prim.attributes.NORMAL);
  const index = accessor(gltf, binary, prim.indices).data;
  const jidx = accessor(gltf, binary, prim.attributes.JOINTS_0);
  const jw = accessor(gltf, binary, prim.attributes.WEIGHTS_0);
  const bind = joints.map((j) => rigid(w(j)));
  const names = joints.map((j) => nodes[j].name || `bone${j}`);
  const parents = joints.map((j) => {
    let p = parent[j];
    while (p >= 0 && !slot.has(p)) p = parent[p];
    return slot.has(p) ? slot.get(p) : -1;
  });
  const root = bind[names.indexOf("XRHand_Wrist")];
  const C = root.R.clone().transpose();
  const t = root.p.clone().applyMatrix3(C).negate();
  const bindR = bind.map((b) => C.clone().multiply(b.R));
  const bindP = bind.map((b) => b.p.clone().applyMatrix3(C).add(t));
  const skinRot = new THREE.Matrix3().setFromMatrix4(skinMat);
  const n = pos.count;
  const vertices = new Float64Array(n * 3);
  const normals = new Float64Array(n * 3);
  const v = new THREE.Vector3();
  for (let i = 0; i < n; i++) {
    v.fromArray(pos.data, i * 3).applyMatrix4(skinMat).applyMatrix3(C).add(t).toArray(vertices, i * 3);
    v.fromArray(nrm.data, i * 3).applyMatrix3(skinRot).applyMatrix3(C).normalize().toArray(normals, i * 3);
  }
  const skinIndex = new Uint16Array(n * 4);
  const skinWeight = new Float32Array(n * 4);
  for (let i = 0; i < n; i++) {
    let sum = 0;
    for (let c = 0; c < 4; c++) sum += jw.data[i * jw.width + c];
    for (let c = 0; c < 4; c++) {
      skinIndex[i * 4 + c] = jidx.data[i * jidx.width + c];
      skinWeight[i * 4 + c] = jw.data[i * jw.width + c] / Math.max(sum, 1e-12);
    }
  }
  const order = [];
  const seen = new Set();
  const visit = (k) => {
    if (seen.has(k)) return;
    if (parents[k] >= 0) visit(parents[k]);
    seen.add(k);
    order.push(k);
  };
  joints.forEach((_, k) => visit(k));
  return {
    names, parents, order, bindR, bindP,
    jointOf: names.map((nm) => (nm in DRIVEN_BONES ? DRIVEN_BONES[nm] : -1)),
    vertices, normals, skinIndex, skinWeight, index: Uint32Array.from(index),
  };
}

export function boneTransforms(hand, quats, positions) {
  const count = hand.names.length;
  const R = new Array(count);
  const p = new Array(count);
  const q = new THREE.Quaternion();
  const m4 = new THREE.Matrix4();
  for (const k of hand.order) {
    const j = hand.jointOf[k];
    const par = hand.parents[k];
    if (j >= 0) {
      q.set(quats[j * 4], quats[j * 4 + 1], quats[j * 4 + 2], quats[j * 4 + 3]).normalize();
      R[k] = new THREE.Matrix3().setFromMatrix4(m4.makeRotationFromQuaternion(q));
      p[k] = new THREE.Vector3(positions[j * 3], positions[j * 3 + 1], positions[j * 3 + 2]);
    } else if (par < 0) {
      R[k] = hand.bindR[k].clone();
      p[k] = hand.bindP[k].clone();
    } else {
      const rel = R[par].clone().multiply(hand.bindR[par].clone().transpose());
      R[k] = rel.clone().multiply(hand.bindR[k]);
      p[k] = hand.bindP[k].clone().sub(hand.bindP[par]).applyMatrix3(rel).add(p[par]);
    }
  }
  return { R, p };
}

export function skinInto(hand, quats, positions, bindPos, bindNrm, skinIndex, skinWeight, outPos, outNrm) {
  const { R, p } = boneTransforms(hand, quats, positions);
  const count = R.length;
  const M = new Float64Array(count * 9);
  const shift = new Float64Array(count * 3);
  for (let k = 0; k < count; k++) {
    const m = R[k].clone().multiply(hand.bindR[k].clone().transpose()).elements;
    const b = hand.bindP[k];
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) M[k * 9 + r * 3 + c] = m[c * 3 + r];
      shift[k * 3 + r] = p[k].getComponent(r) - (M[k * 9 + r * 3] * b.x + M[k * 9 + r * 3 + 1] * b.y + M[k * 9 + r * 3 + 2] * b.z);
    }
  }
  const n = bindPos.length / 3;
  for (let i = 0; i < n; i++) {
    const x = bindPos[i * 3];
    const y = bindPos[i * 3 + 1];
    const z = bindPos[i * 3 + 2];
    const nx = bindNrm[i * 3];
    const ny = bindNrm[i * 3 + 1];
    const nz = bindNrm[i * 3 + 2];
    let px = 0;
    let py = 0;
    let pz = 0;
    let qx = 0;
    let qy = 0;
    let qz = 0;
    for (let c = 0; c < 4; c++) {
      const wgt = skinWeight[i * 4 + c];
      if (!wgt) continue;
      const k = skinIndex[i * 4 + c];
      const o = k * 9;
      px += wgt * (M[o] * x + M[o + 1] * y + M[o + 2] * z + shift[k * 3]);
      py += wgt * (M[o + 3] * x + M[o + 4] * y + M[o + 5] * z + shift[k * 3 + 1]);
      pz += wgt * (M[o + 6] * x + M[o + 7] * y + M[o + 8] * z + shift[k * 3 + 2]);
      qx += wgt * (M[o] * nx + M[o + 1] * ny + M[o + 2] * nz);
      qy += wgt * (M[o + 3] * nx + M[o + 4] * ny + M[o + 5] * nz);
      qz += wgt * (M[o + 6] * nx + M[o + 7] * ny + M[o + 8] * nz);
    }
    outPos[i * 3] = px;
    outPos[i * 3 + 1] = py;
    outPos[i * 3 + 2] = pz;
    const len = Math.hypot(qx, qy, qz) || 1;
    outNrm[i * 3] = qx / len;
    outNrm[i * 3 + 1] = qy / len;
    outNrm[i * 3 + 2] = qz / len;
  }
}
