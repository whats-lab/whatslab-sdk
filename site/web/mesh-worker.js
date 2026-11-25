import { ColorManagement, Mesh, SRGBColorSpace } from "https://cdn.jsdelivr.net/npm/three@0.186.1/+esm";
import { STLLoader } from "https://cdn.jsdelivr.net/npm/three@0.186.1/examples/jsm/loaders/STLLoader.js/+esm";
import { OBJLoader } from "https://cdn.jsdelivr.net/npm/three@0.186.1/examples/jsm/loaders/OBJLoader.js/+esm";
import { MTLLoader } from "https://cdn.jsdelivr.net/npm/three@0.186.1/examples/jsm/loaders/MTLLoader.js/+esm";
import { ColladaLoader } from "https://cdn.jsdelivr.net/npm/three@0.186.1/examples/jsm/loaders/ColladaLoader.js/+esm";
import { GLTFLoader } from "https://cdn.jsdelivr.net/npm/three@0.186.1/examples/jsm/loaders/GLTFLoader.js/+esm";
import { toCreasedNormals } from "https://cdn.jsdelivr.net/npm/three@0.186.1/examples/jsm/utils/BufferGeometryUtils.js/+esm";
import { DOMParser } from "https://cdn.jsdelivr.net/npm/@xmldom/xmldom@0.8.10/+esm";

self.DOMParser = DOMParser;

const CREASE = (35 * Math.PI) / 180;
const loads = new Map();

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

function rawColor(c, linear) {
  const out = c.clone();
  if (linear) ColorManagement.workingToColorSpace(out, SRGBColorSpace);
  return out.toArray();
}

function texture(t, transfer) {
  if (!t || !t.image || typeof ImageBitmap === "undefined" || !(t.image instanceof ImageBitmap)) return null;
  transfer.push(t.image);
  return {
    image: t.image,
    flipY: t.flipY,
    colorSpace: t.colorSpace,
    wrapS: t.wrapS,
    wrapT: t.wrapT,
    repeat: t.repeat.toArray(),
    offset: t.offset.toArray(),
  };
}

function material(m, linear, mtl, transfer) {
  const info = mtl && m.name && mtl.materialsInfo[m.name];
  if (info) {
    const kd = info.kd ? info.kd.map(Number) : [1, 1, 1];
    const d = info.d !== undefined ? Number(info.d) : 1;
    return { kind: "basic", color: kd, opacity: d, transparent: d < 1, map: null, mapFile: info.map_kd || null };
  }
  return {
    kind: m.isMeshStandardMaterial ? "standard" : "basic",
    color: m.color ? rawColor(m.color, linear && !m.isMeshStandardMaterial) : [1, 1, 1],
    opacity: m.opacity,
    transparent: m.transparent,
    side: m.side,
    vertexColors: m.vertexColors,
    metalness: m.metalness,
    roughness: m.roughness,
    emissive: m.emissive ? m.emissive.toArray() : null,
    map: texture(m.map, transfer),
  };
}

function pack(root, own, linear, mtl) {
  root.updateMatrixWorld(true);
  const parts = [];
  const transfer = [];
  root.traverse((o) => {
    if (!o.isMesh) return;
    let g = o.geometry;
    if (faceted(g)) g = toCreasedNormals(g, CREASE);
    const attributes = {};
    for (const [name, a] of Object.entries(g.attributes)) {
      const array = a.isInterleavedBufferAttribute ? a.clone().array : a.array.slice();
      attributes[name] = { array, itemSize: a.itemSize, normalized: a.normalized };
      transfer.push(array.buffer);
    }
    const index = g.index ? g.index.array.slice() : null;
    if (index) transfer.push(index.buffer);
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    parts.push({
      attributes,
      index,
      groups: g.groups.map((x) => ({ ...x })),
      matrix: o.matrixWorld.toArray(),
      multi: Array.isArray(o.material),
      materials: own ? mats.map((m) => m && material(m, linear, mtl, transfer)) : null,
    });
  });
  return { parts, transfer };
}

function resolve(load, raw) {
  const u = new URL(raw).href;
  if (!u.startsWith(load.robotsBase)) return u;
  const rel = decodeURIComponent(u.slice(load.robotsBase.length));
  const entry = load.meshes.files[rel.slice(rel.indexOf("/") + 1)];
  return entry ? load.meshes.base + entry[0].split("/").map(encodeURIComponent).join("/") : u;
}

function get(load, url, kind) {
  return fetch(resolve(load, url), { signal: load.controller.signal, cache: "force-cache" })
    .then((r) => (r.ok ? r[kind]() : null))
    .then((body) => body, () => null);
}

function parse(load, url, ext, body) {
  const base = url.slice(0, url.lastIndexOf("/") + 1);
  if (ext === "stl") return Promise.resolve(pack(new Mesh(new STLLoader().parse(body)), false, false, null));
  if (ext === "dae") {
    const yUp = body.replace(/<up_axis>\s*Z_UP\s*<\/up_axis>/, "<up_axis>Y_UP</up_axis>");
    return Promise.resolve(pack(new ColladaLoader().parse(yUp, base).scene, true, true, null));
  }
  if (ext === "glb" || ext === "gltf") {
    return new Promise((done) => {
      new GLTFLoader().parse(body, base, (g) => done(pack(g.scene, true, false, null)), () => done(null));
    });
  }
  if (ext === "obj") {
    const lib = body.match(/^mtllib\s+(.+)$/m);
    const mtl = lib ? get(load, base + lib[1].trim(), "text") : Promise.resolve(null);
    return mtl.then((text) => {
      const info = text ? new MTLLoader().parse(text, base) : null;
      return pack(new OBJLoader().parse(body), Boolean(info), true, info);
    });
  }
  return Promise.resolve(null);
}

self.onmessage = (e) => {
  const msg = e.data;
  if (msg.type === "start") {
    loads.set(msg.load, { controller: new AbortController(), meshes: msg.meshes, robotsBase: msg.robotsBase });
  } else if (msg.type === "cancel") {
    const load = loads.get(msg.load);
    if (load) load.controller.abort();
    loads.delete(msg.load);
  } else if (msg.type === "mesh") {
    const load = loads.get(msg.load);
    if (!load) return;
    const ext = msg.url.split(".").pop().toLowerCase();
    get(load, msg.url, ext === "dae" || ext === "obj" ? "text" : "arrayBuffer")
      .then((body) => {
        if (!loads.has(msg.load)) return null;
        self.postMessage({ type: "fetched", load: msg.load, req: msg.req, ok: Boolean(body) });
        return body ? parse(load, msg.url, ext, body) : null;
      })
      .then((res) => {
        if (!loads.has(msg.load)) return;
        if (!res) {
          self.postMessage({ type: "result", load: msg.load, req: msg.req, parts: null });
          return;
        }
        self.postMessage({ type: "result", load: msg.load, req: msg.req, parts: res.parts }, res.transfer);
      }, (err) => {
        self.postMessage({ type: "result", load: msg.load, req: msg.req, parts: null, error: String(err && err.message ? err.message : err) });
      });
  }
};

