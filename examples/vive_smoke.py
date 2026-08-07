from __future__ import annotations

import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.viz import get_server

M = np.eye(3)


def _wxyz(R: np.ndarray) -> tuple:
    q = Rotation.from_matrix(R).as_quat()
    return (float(q[3]), float(q[0]), float(q[1]), float(q[2]))


def _canonical(pose) -> tuple:
    p = np.array([pose.Pos[0], pose.Pos[1], pose.Pos[2]], dtype=float)
    q_xyzw = np.array([pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]], dtype=float)
    p_c = M @ p
    R_c = M @ Rotation.from_quat(q_xyzw).as_matrix() @ M.T
    return p_c, R_c


def main() -> None:
    try:
        import pysurvive
    except ImportError:
        sys.exit("pysurvive 미설치. `pip install pysurvive` (라이트하우스 셋업 선행).")

    ctx = pysurvive.SimpleContext(sys.argv)
    srv = get_server()

    print("감지된 객체 (이 serial 을 trackers={side: ...} 에 사용):")
    for obj in ctx.Objects():
        print(f"  · {obj.Name()}")

    capture = {"pending": False}
    btn = srv.gui.add_button("Capture home (calibrate)")
    btn.on_click(lambda _: capture.__setitem__("pending", True))
    readout = srv.gui.add_markdown("*(트래커 대기 중)*")

    frames: dict = {}
    ref: dict = {}
    latest: dict = {}

    def _ensure(name: str):
        if name in frames:
            return frames[name]
        h = {
            "track": srv.scene.add_frame(f"/tracker/{name}", show_axes=True,
                                         axes_length=0.15, axes_radius=0.006),
            "home": srv.scene.add_frame(f"/home/{name}", show_axes=True,
                                        axes_length=0.1, axes_radius=0.004,
                                        visible=False),
            "rel": srv.scene.add_frame(f"/rel/{name}", show_axes=True,
                                       axes_length=0.12, axes_radius=0.006),
        }
        frames[name] = h
        return h

    last = 0.0
    while ctx.Running():
        obj = ctx.NextUpdated()
        if obj is None:
            continue
        name = obj.Name()
        p_c, R_c = _canonical(obj.Pose()[0])
        latest[name] = (p_c, R_c)
        h = _ensure(name)

        h["track"].position = tuple(p_c)
        h["track"].wxyz = _wxyz(R_c)

        if capture["pending"]:
            ref[name] = (p_c.copy(), R_c.T.copy())
            h["home"].position = tuple(p_c)
            h["home"].wxyz = _wxyz(R_c)
            h["home"].visible = True

        if name in ref:
            p0_c, R0_inv = ref[name]
            d_pos = R0_inv @ (p_c - p0_c)
            R_rel = R0_inv @ R_c
            h["rel"].position = tuple(d_pos)
            h["rel"].wxyz = _wxyz(R_rel)

        if capture["pending"]:
            capture["pending"] = False

        now = time.monotonic()
        if now - last >= 0.1:
            last = now
            lines = []
            for nm, (p0_c, R0_inv) in ref.items():
                pc, Rc = latest[nm]
                dp = R0_inv @ (pc - p0_c)
                rpy = Rotation.from_matrix(R0_inv @ Rc).as_euler("xyz", degrees=True)
                lines.append(f"**{nm}**  Δpos(m)={dp.round(3).tolist()}  "
                             f"Δrpy(°)={rpy.round(1).tolist()}")
            readout.content = "\n\n".join(lines) or "*(Capture home 눌러 기준 잡기)*"


if __name__ == "__main__":
    main()
