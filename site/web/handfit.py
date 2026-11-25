import json
import struct

import numpy as np

FINGERS = ("index", "middle", "ring", "pinky")
BONES = (
    "wrist",
    "thumb-metacarpal",
    "thumb-phalanx-proximal",
    "thumb-phalanx-distal",
    "thumb-tip",
    *(
        f"{f}-finger-{b}"
        for f in FINGERS
        for b in (
            "metacarpal",
            "phalanx-proximal",
            "phalanx-intermediate",
            "phalanx-distal",
            "tip",
        )
    ),
)
PARENTS = np.array(
    [-1, 0, 1, 2, 3] + [p for k in range(4) for p in (0, *range(5 + 5 * k, 9 + 5 * k))]
)
MP_JOINTS = np.array([0, 1, 2, 3, 4] + [5 * k + s for k in range(1, 5) for s in (1, 2, 3, 4)])
FLEX, ABD, ROLL = 0, 1, 2
DOFS = (
    (1, FLEX, -0.6, 0.9),
    (1, ABD, -0.6, 0.9),
    (1, ROLL, -0.6, 0.6),
    (2, FLEX, -0.3, 1.2),
    (2, ABD, -0.25, 0.25),
    (3, FLEX, -0.4, 1.5),
    (15, FLEX, 0.0, 0.3),
    (20, FLEX, 0.0, 0.35),
    *(
        dof
        for k in range(4)
        for dof in (
            (6 + 5 * k, FLEX, -0.35, 1.6),
            (6 + 5 * k, ABD, -0.35, 0.35),
            (7 + 5 * k, FLEX, -0.1, 1.9),
            (8 + 5 * k, FLEX, -0.1, 1.4),
        )
    ),
)
FOCAL = 0.785
W_DEPTH = 0.05
W_ZERO = 0.004
W_SMOOTH = 0.01
EPS = 1e-5
ITERS = 6
RESET = 0.03


def quat_to_matrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(m):
    w = np.sqrt(np.maximum(0.0, 1 + m[..., 0, 0] + m[..., 1, 1] + m[..., 2, 2])) / 2
    x = np.sqrt(np.maximum(0.0, 1 + m[..., 0, 0] - m[..., 1, 1] - m[..., 2, 2])) / 2
    y = np.sqrt(np.maximum(0.0, 1 - m[..., 0, 0] + m[..., 1, 1] - m[..., 2, 2])) / 2
    z = np.sqrt(np.maximum(0.0, 1 - m[..., 0, 0] - m[..., 1, 1] + m[..., 2, 2])) / 2
    x = np.copysign(x, m[..., 2, 1] - m[..., 1, 2])
    y = np.copysign(y, m[..., 0, 2] - m[..., 2, 0])
    z = np.copysign(z, m[..., 1, 0] - m[..., 0, 1])
    return np.stack([x, y, z, w], axis=-1)


def rotvec(v):
    v = np.asarray(v, dtype=float)
    a = np.linalg.norm(v, axis=-1, keepdims=True)
    k = v / np.maximum(a, 1e-12)
    kx, ky, kz = k[..., 0], k[..., 1], k[..., 2]
    zero = np.zeros_like(kx)
    skew = np.stack([zero, -kz, ky, kz, zero, -kx, -ky, kx, zero], axis=-1)
    skew = skew.reshape(v.shape[:-1] + (3, 3))
    s, c = np.sin(a)[..., None], np.cos(a)[..., None]
    return np.eye(3) + s * skew + (1 - c) * (skew @ skew)


def matrix_log(m):
    c = np.clip((np.trace(m) - 1) / 2, -1.0, 1.0)
    a = np.arccos(c)
    w = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    if a < 1e-6:
        return w / 2
    if a > np.pi - 1e-4:
        d = np.sqrt(np.maximum((np.diag(m) + 1) / 2, 0.0))
        d = d * np.where(w >= 0, 1.0, -1.0)
        return a * d / max(np.linalg.norm(d), 1e-12)
    return a * w / (2 * np.sin(a))


def axis_rotation(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    o, z = np.ones_like(angle), np.zeros_like(angle)
    if axis == FLEX:
        rows = [o, z, z, z, c, -s, z, s, c]
    elif axis == ABD:
        rows = [c, z, s, z, o, z, -s, z, c]
    else:
        rows = [c, -s, z, s, c, z, z, z, o]
    return np.stack(rows, axis=-1).reshape(angle.shape + (3, 3))


def glb_bind(data):
    length = struct.unpack_from("<I", data, 12)[0]
    nodes = {n.get("name"): n for n in json.loads(bytes(data[20 : 20 + length]))["nodes"]}
    pos = np.array([nodes[b]["translation"] for b in BONES], dtype=float)
    rot = np.array([quat_to_matrix(np.array(nodes[b]["rotation"], dtype=float)) for b in BONES])
    return pos, rot


def palm_frame(wrist, index, middle, pinky, side):
    z = wrist - middle
    z = z / np.linalg.norm(z)
    across = pinky - index if side == "right" else index - pinky
    x = across - z * (across @ z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


class HandFit:
    def __init__(self, glb, side):
        pos, rot = glb_bind(glb)
        self.side = side
        rest = range(1, len(BONES))
        self.rel = np.array([np.eye(3)] + [rot[PARENTS[j]].T @ rot[j] for j in rest])
        self.off = np.array(
            [np.zeros(3)] + [rot[PARENTS[j]].T @ (pos[j] - pos[PARENTS[j]]) for j in rest]
        )
        self.root = rot[0]
        self.levels = []
        depth = np.zeros(len(BONES), dtype=int)
        for j in range(1, len(BONES)):
            depth[j] = depth[PARENTS[j]] + 1
        for d in range(1, depth.max() + 1):
            self.levels.append(np.flatnonzero(depth == d))
        self.joint = np.array([d[0] for d in DOFS])
        self.axis = np.array([d[1] for d in DOFS])
        self.lo = np.array([d[2] for d in DOFS])
        self.hi = np.array([d[3] for d in DOFS])
        self.n = len(DOFS)
        self.bones = [(int(PARENTS[j]), int(j)) for j in MP_JOINTS if PARENTS[j] in MP_JOINTS]
        self.mp_of = {int(j): k for k, j in enumerate(MP_JOINTS)}
        p0, _ = self.fk(np.zeros((1, self.n)))
        self.bind_pos = p0[0]
        self.x = None
        self.uv = np.zeros((len(BONES), 2))

    def fk(self, q):
        b = q.shape[0]
        local = np.broadcast_to(self.rel, (b,) + self.rel.shape).copy()
        for axis in (ROLL, FLEX, ABD):
            sel = self.axis == axis
            if not sel.any():
                continue
            ang = q[:, sel] * (-1.0 if axis == FLEX else 1.0)
            m = axis_rotation(axis, ang)
            idx = self.joint[sel]
            local[:, idx] = local[:, idx] @ m
        rot = np.empty((b, len(BONES), 3, 3))
        pos = np.empty((b, len(BONES), 3))
        rot[:, 0] = self.root
        pos[:, 0] = 0.0
        for lvl in self.levels:
            par = PARENTS[lvl]
            pos[:, lvl] = pos[:, par] + np.einsum("bkij,kj->bki", rot[:, par], self.off[lvl])
            rot[:, lvl] = rot[:, par] @ local[:, lvl]
        return pos, rot

    def project(self, x, aspect):
        cam = rotvec(x[:, :3])
        pos, rot = self.fk(np.clip(x[:, 6:], self.lo, self.hi))
        pts = np.einsum("bij,bkj->bki", cam, pos)
        pts = pts + x[:, None, 3:6]
        z = np.maximum(pts[..., 2], 1e-3)
        uv = np.stack(
            [FOCAL * pts[..., 0] / z + 0.5, FOCAL * aspect * pts[..., 1] / z + 0.5], axis=-1
        )
        return uv, pts, cam, rot

    def residual(self, x, world, image, aspect, q_prev):
        uv, pts, _, _ = self.project(x, aspect)
        parts = [(uv[:, MP_JOINTS] - image[None]).reshape(len(x), -1)]
        par = np.array([p for p, _ in self.bones])
        chi = np.array([j for _, j in self.bones])
        d = pts[:, chi] - pts[:, par]
        dz = d[..., 2] / np.linalg.norm(d, axis=-1)
        w = world[[self.mp_of[j] for j in chi]] - world[[self.mp_of[p] for p in par]]
        wz = w[:, 2] / np.linalg.norm(w, axis=-1)
        parts.append(W_DEPTH * (dz - wz[None]))
        parts.append(W_ZERO * x[:, 6:])
        if q_prev is not None:
            parts.append(W_SMOOTH * (x[:, 6:] - q_prev[None]))
        return np.concatenate(parts, axis=1)

    def initial(self, world, image, aspect):
        target = palm_frame(world[0], world[5], world[9], world[17], self.side)
        model = palm_frame(*(self.bind_pos[j] for j in (0, 6, 11, 21)), self.side)
        x = np.zeros(6 + self.n)
        x[:3] = matrix_log(target @ model.T)
        pts = self.bind_pos[MP_JOINTS] @ rotvec(x[:3]).T
        span_img = np.linalg.norm(image - image[0], axis=1).sum()
        span_mod = np.linalg.norm(pts[:, :2] - pts[0, :2], axis=1).sum()
        z = FOCAL * span_mod / max(span_img, 1e-9)
        x[5] = z
        x[3] = (image[0, 0] - 0.5) * z / FOCAL - pts[0, 0]
        x[4] = (image[0, 1] - 0.5) * z / (FOCAL * aspect) - pts[0, 1]
        return x

    def optimize(self, x, world, image, aspect, q_prev, iters):
        lam = 1e-3
        r = self.residual(x[None], world, image, aspect, q_prev)[0]
        eye = np.eye(len(x)) * EPS
        for _ in range(iters):
            rs = self.residual(x[None] + eye, world, image, aspect, q_prev)
            jac = (rs - r[None]).T / EPS
            g, hess = jac.T @ r, jac.T @ jac
            accepted = False
            while lam <= 1e4:
                step = np.linalg.solve(hess + lam * np.diag(np.diag(hess) + 1e-9), -g)
                xn = x + step
                xn[6:] = np.clip(xn[6:], self.lo, self.hi)
                rn = self.residual(xn[None], world, image, aspect, q_prev)[0]
                if rn @ rn < r @ r:
                    x, r, lam, accepted = xn, rn, max(lam * 0.3, 1e-6), True
                    break
                lam *= 10
            if not accepted or np.abs(step).max() < 1e-6:
                break
        return x

    def fit(self, world, image, aspect):
        world = np.asarray(world, dtype=float).reshape(21, 3)
        image = np.asarray(image, dtype=float).reshape(21, 2)
        fresh = self.x is None
        x = self.initial(world, image, aspect) if fresh else self.x.copy()
        q_prev = None if fresh else self.x[6:].copy()
        x = self.optimize(x, world, image, aspect, q_prev, ITERS * (3 if fresh else 1))
        uv, pts, cam, rot = self.project(x[None], aspect)
        err = float(np.linalg.norm(uv[0, MP_JOINTS] - image, axis=1).mean())
        if not np.isfinite(err):
            self.x = None
            return None
        if err > RESET and not fresh:
            self.x = None
            return self.fit(world, image, aspect)
        self.x = x
        self.uv = uv[0]
        return self.xr(pts[0], cam[0] @ rot[0]), err

    def xr(self, pts, rot):
        out = np.zeros((26, 8))
        out[1:, :4] = matrix_to_quat(rot)
        out[1:, 4:7] = pts
        out[0, :4] = out[1, :4]
        out[0, 4:7] = (pts[0] + pts[11]) / 2
        out[:, 7] = 0.01
        return out
