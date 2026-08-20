from __future__ import annotations

import logging
from typing import List, Optional, Sequence

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


def xyzrpy_to_mat(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    mat[:3, 3] = np.array([x, y, z])
    return mat


class _ArmSolverBase:

    proj_rcond = 1e-6

    def _null_projector(self, WJ: np.ndarray, In: np.ndarray) -> np.ndarray:
        wi = 1.0 / self._joint_w
        _, S, Vt = np.linalg.svd(WJ * wi, full_matrices=False)
        keep = S > (self.proj_rcond * S[0] if S.size and S[0] > 0.0 else 0.0)
        if not np.any(keep):
            return In
        V = Vt[keep]
        return wi[:, None] * (In - V.T @ V) * self._joint_w

    def __init__(
        self,
        urdf_path: str,
        locked_joints: List[str],
        ee_parent_joint: str,
        ee_frame_name: str = "ee",
        tool_pre_rot_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        tool_translation_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        w_pos: float = 20.0,
        w_ori: float = 1.0,
        max_iter: int = 50,
        tol: float = 1e-4,
    ):
        m_full = pin.buildModelFromUrdf(urdf_path)
        lock_ids, seen = [], set()
        for name in locked_joints:
            if not m_full.existJointName(name):
                continue
            jid = m_full.getJointId(name)
            if jid <= 0 or jid in seen:
                continue
            seen.add(jid)
            lock_ids.append(jid)
        model = (pin.buildReducedModel(m_full, lock_ids, pin.neutral(m_full))
                 if lock_ids else m_full)

        first = xyzrpy_to_mat(0.0, 0.0, 0.0, tool_pre_rot_rpy[0], tool_pre_rot_rpy[1], tool_pre_rot_rpy[2])
        second = xyzrpy_to_mat(tool_translation_xyz[0], tool_translation_xyz[1], tool_translation_xyz[2], 0.0, 0.0, 0.0)
        ee_mat = first @ second
        quat = Rotation.from_matrix(ee_mat[:3, :3]).as_quat()
        local = pin.SE3(pin.Quaternion(quat[3], quat[0], quat[1], quat[2]),
                        np.array(ee_mat[:3, 3]))
        if model.existJointName(ee_parent_joint):
            jid = model.getJointId(ee_parent_joint)
            placement = local
        else:
            pf = model.frames[model.getFrameId(ee_parent_joint)]
            jp = getattr(pf, "parentJoint", None)
            jid = int(jp if jp is not None else pf.parent)
            placement = pf.placement * local
        try:
            parent_frame = model.getFrameId(model.names[jid])
            frame = pin.Frame(ee_frame_name, jid, parent_frame,
                              placement, pin.FrameType.OP_FRAME)
        except Exception:
            frame = pin.Frame(ee_frame_name, jid, placement, pin.FrameType.OP_FRAME)
        model.addFrame(frame)

        self.model = model
        self.ee_id = model.getFrameId(ee_frame_name)
        self._finish_setup(w_pos, w_ori, max_iter, tol)

    def _finish_setup(self, w_pos, w_ori, max_iter, tol):
        self.w_pos = float(w_pos)
        self.w_ori = float(w_ori)
        self.max_iter = int(max_iter)
        self.tol = float(tol)

        w = np.array([w_pos] * 3 + [w_ori] * 3, dtype=float)
        self._task_w = w / max(w.max(), 1e-9)
        self._damp = 1e-2
        self._lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                            self.model.lowerPositionLimit, -np.pi)
        self._hi = np.where(np.isfinite(self.model.upperPositionLimit),
                            self.model.upperPositionLimit, np.pi)
        self.limit_margin = 0.10
        self.k_limit = 0.30
        self._smooth = 0.2

        self._joint_w = np.ones(self.model.nv)
        self._q_neutral = self._mid_range_config()
        self.data = self.model.createData()
        self.init_data = np.zeros(self.model.nq)
        self.history_data = np.zeros(self.model.nq)
        self._fk_data = self.model.createData()

    def _mid_range_config(self) -> np.ndarray:
        q = pin.neutral(self.model)
        lo = self.model.lowerPositionLimit
        hi = self.model.upperPositionLimit
        for j in self.model.joints:
            if j.nq != 1 or j.nv != 1:
                continue
            i = j.idx_q
            if np.isfinite(lo[i]) and np.isfinite(hi[i]):
                q[i] = 0.5 * (lo[i] + hi[i])
        return q

    @classmethod
    def from_appended(
        cls, arm_urdf: str, hand_urdf: str, attach_frame: str, ee_link: str,
        mount_xyz: Sequence[float] = (0.0, 0.0, 0.0), mount_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        locked_joints: Optional[List[str]] = None,
        w_pos: float = 20.0, w_ori: float = 10.0,
        max_iter: int = 50, tol: float = 1e-4,
        ee_local_rpy: Sequence[float] = (0.0, -np.pi / 2, np.pi / 2),
    ) -> "ArmIK":
        self = cls.__new__(cls)
        m_arm = pin.buildModelFromUrdf(arm_urdf)
        m_hand = pin.buildModelFromUrdf(hand_urdf)
        arm_frame_names = {f.name for f in m_arm.frames}
        for f in m_hand.frames:
            if f.name in arm_frame_names and f.name != ee_link:
                f.name = f.name + "_hand"
        fid = m_arm.getFrameId(attach_frame)
        aMb = pin.SE3(Rotation.from_euler("xyz", list(mount_rpy)).as_matrix(),
                      np.array(mount_xyz, dtype=float))
        combined = pin.appendModel(m_arm, m_hand, fid, aMb)
        ee_frame = combined.frames[combined.getFrameId(ee_link)]
        j_ee = getattr(ee_frame, "parentJoint", None)
        if j_ee is None:
            j_ee = ee_frame.parent
        chain = {combined.names[i] for i in combined.supports[int(j_ee)]}
        keep = (chain - {"universe"}) - set(locked_joints or [])
        lock_ids = [combined.getJointId(n) for n in combined.names
                    if n != "universe" and n not in keep]
        reduced = pin.buildReducedModel(combined, lock_ids, pin.neutral(combined))

        orig_ee_id = reduced.getFrameId(ee_link)

        local_rot = Rotation.from_euler("xyz", list(ee_local_rpy)).as_matrix()
        reduced.frames[orig_ee_id].placement = (
            reduced.frames[orig_ee_id].placement * pin.SE3(local_rot, np.zeros(3)))

        self.model = reduced
        self.data = reduced.createData()
        self.ee_id = reduced.getFrameId(ee_link)
        self._finish_setup(w_pos, w_ori, max_iter, tol)
        return self

    def solve(self, target_pose: np.ndarray, safe: bool = True) -> np.ndarray:
        raise NotImplementedError

    @property
    def nq(self) -> int:
        return self.model.nq

    def active_joint_names(self) -> List[str]:
        return [n for n in self.model.names if n != "universe"]

    def sync_state(self, q_current: Sequence[float]) -> None:
        q = np.array(q_current, dtype=float)
        if q.shape[0] == self.nq:
            self.init_data = q
            self.history_data = q

    def set_joint_weights(self, weights: Optional[dict]) -> None:
        w = np.ones(self.model.nv)
        for name, cost in (weights or {}).items():
            if not self.model.existJointName(name):
                raise ValueError(f"joint_weights 에 없는 관절: {name}")
            c = float(cost)
            if c <= 0.0:
                raise ValueError(f"joint_weights[{name}] 는 양수여야 한다: {c}")
            j = self.model.joints[self.model.getJointId(name)]
            w[j.idx_v:j.idx_v + j.nv] = c
        self._joint_w = w

    def _damped_pinv(self, WJ: np.ndarray, damp2: float) -> np.ndarray:
        wi = 1.0 / self._joint_w
        I = np.eye(WJ.shape[0])
        return (wi[:, None] * WJ.T) @ np.linalg.inv((WJ * wi) @ WJ.T + damp2 * I)

    def _error_and_jac(self, q: np.ndarray, T: np.ndarray):
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        iMd = self.data.oMf[self.ee_id].actInv(pin.SE3(np.asarray(T, dtype=float)))
        e = pin.log6(iMd).vector
        Jf = pin.computeFrameJacobian(self.model, self.data, q, self.ee_id, pin.LOCAL)
        J = -pin.Jlog6(iMd.inverse()) @ Jf

        return e, J

    def _limit_gradient(self, q: np.ndarray) -> np.ndarray:
        m = self.limit_margin
        g = np.zeros_like(q)
        low_head = q - self._lo
        high_head = self._hi - q
        near_low = low_head < m
        near_high = high_head < m
        g[near_low] += (m - low_head[near_low]) / m
        g[near_high] -= (m - high_head[near_high]) / m
        return g

    def _soft_limit_scale(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        m = self.limit_margin
        out = dq.copy()
        up = dq > 0
        dn = dq < 0
        head = np.where(up, self._hi - q, np.where(dn, q - self._lo, m))
        scale = np.clip(head / m, 0.0, 1.0)
        near = head < m
        out[near] *= scale[near]
        return out

    def converge(self, target_pose: np.ndarray, q0: np.ndarray) -> np.ndarray:
        T = np.asarray(target_pose, dtype=float)
        q = np.array(q0, dtype=float)
        w = self._task_w
        damp2 = self._damp * self._damp
        In = np.eye(self.model.nq)
        for _ in range(self.max_iter):
            e, J = self._error_and_jac(q, T)
            if np.linalg.norm(e) < self.tol:
                break
            we = w * e
            WJ = w[:, None] * J
            Jpinv = self._damped_pinv(WJ, damp2)
            dq_task = -Jpinv @ we
            N = self._null_projector(WJ, In)
            dq = dq_task + N @ (self.k_limit * self._limit_gradient(q))
            dq = self._soft_limit_scale(q, dq)
            n = np.linalg.norm(dq)
            if n > 1.0:
                dq *= 1.0 / n
            q = pin.integrate(self.model, q, dq)
        return np.clip(q, self._lo, self._hi)

    @property
    def q_neutral(self) -> np.ndarray:
        return self._q_neutral.copy()

    def _warn_once(self, exc: Exception) -> None:
        key = f"{type(exc).__name__}: {exc}"
        seen = getattr(self, "_warned", None)
        if seen is None:
            seen = self._warned = set()
        if key not in seen:
            seen.add(key)
            logger.warning("IK solve 예외 — 직전 자세 유지 (추종 정지처럼 보임): %s", key)

    def fk(self, q: np.ndarray) -> np.ndarray:
        pin.framesForwardKinematics(self.model, self._fk_data, np.asarray(q, dtype=float))
        return self._fk_data.oMf[self.ee_id].homogeneous.copy()

    def frame_pose(self, frame_name: str, q: np.ndarray) -> np.ndarray:
        pin.framesForwardKinematics(self.model, self._fk_data, np.asarray(q, dtype=float))
        return self._fk_data.oMf[self.model.getFrameId(frame_name)].homogeneous.copy()

    def pose_error(self, q: np.ndarray, target_pose: np.ndarray) -> tuple:
        T = self.fk(q)
        pos_err = float(np.linalg.norm(T[:3, 3] - target_pose[:3, 3]))
        R = T[:3, :3].T @ target_pose[:3, :3]
        ori_err = float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
        return pos_err, ori_err

    def solve_robust(
        self,
        target_pose: np.ndarray,
        restarts: int = 10,
        pos_tol: float = 1e-3,
        ori_tol: float = 1e-2,
        seed: int = 0,
        q_ref: Optional[Sequence[float]] = None,
        w_dist: float = 0.0,
        dq_near: float = 0.6,
    ) -> np.ndarray:
        q_ref_arr = np.array(self.history_data if q_ref is None else q_ref, dtype=float)
        rng = np.random.default_rng(seed)
        lo, hi = self._lo, self._hi

        best_q = None
        best_score = np.inf
        for q0 in self._restart_seeds(q_ref_arr, restarts, lo, hi, rng, w_dist):
            try:
                q = self.converge(target_pose, q0)
            except Exception as e:
                self._warn_once(e)
                continue
            if not np.all(np.isfinite(q)):
                continue
            pe, oe = self.pose_error(q, target_pose)
            dist = float(np.linalg.norm(q - q_ref_arr)) if w_dist > 0.0 else 0.0
            score = pe + 0.1 * oe + w_dist * dist
            if score < best_score:
                best_score, best_q = score, q
            if pe <= pos_tol and oe <= ori_tol and dist <= dq_near:
                break

        if best_q is None:
            raise RuntimeError("IK 가 어떤 초기값에서도 해를 찾지 못했습니다.")
        self.init_data = best_q
        self.history_data = best_q
        return best_q

    def _restart_seeds(self, q_ref: np.ndarray, restarts: int,
                       lo: np.ndarray, hi: np.ndarray, rng, w_dist: float = 0.0):
        n = max(1, int(restarts))
        yield q_ref.copy()
        span = hi - lo
        n_local = (n - 1) // 3 if w_dist > 0.0 else 0
        for k in range(n_local):
            scale = 0.15 + 0.45 * (k / max(1, n_local - 1))
            yield np.clip(q_ref + scale * span * (rng.random(self.nq) - 0.5), lo, hi)
        for _ in range(max(0, n - 1 - n_local)):
            yield lo + span * rng.random(self.nq)

class ArmIK(_ArmSolverBase):

    def solve(self, target_pose: np.ndarray, safe: bool = True) -> np.ndarray:
        try:
            sol_q = self.converge(target_pose, self.history_data)
            if self._smooth > 0.0:
                sol_q = self._smooth * self.history_data + (1.0 - self._smooth) * sol_q
            if not np.all(np.isfinite(sol_q)):
                raise ValueError("IK 해에 NaN")
        except Exception as e:
            if not safe:
                raise
            self._warn_once(e)
            sol_q = self.history_data.copy()
        self.init_data = sol_q
        self.history_data = sol_q
        return sol_q


class DiffArmIK(_ArmSolverBase):

    iters_per_call = 100
    dp_max = 1.0
    dtheta_max = 0.25
    dq_max_tick = 0.5
    k_posture = 0.05
    sugihara_bias = 1e-4

    def _finish_setup(self, *a, **k):
        super()._finish_setup(*a, **k)
        self._smooth = 0.2
        self.q_posture = self._q_neutral.copy()

    def _rate_limited_target(self, q: np.ndarray, T_goal: np.ndarray) -> np.ndarray:
        T_cur = self.fk(q)
        T = np.asarray(T_goal, dtype=float).copy()
        dp = T[:3, 3] - T_cur[:3, 3]
        n = np.linalg.norm(dp)
        if n > self.dp_max:
            T[:3, 3] = T_cur[:3, 3] + dp * (self.dp_max / n)
        R_rel = T_cur[:3, :3].T @ np.asarray(T_goal, dtype=float)[:3, :3]
        rot = Rotation.from_matrix(R_rel)
        ang = np.linalg.norm(rot.as_rotvec())
        if ang > self.dtheta_max:
            R_step = Rotation.from_rotvec(rot.as_rotvec() * (self.dtheta_max / ang))
            T[:3, :3] = T_cur[:3, :3] @ R_step.as_matrix()
        return T

    def solve(self, target_pose: np.ndarray, safe: bool = True) -> np.ndarray:
        T_goal = np.asarray(target_pose, dtype=float)
        q_start = np.array(self.history_data, dtype=float)
        q = q_start.copy()
        w = self._task_w
        In = np.eye(self.model.nq)
        try:
            T = self._rate_limited_target(q, T_goal)
            for _ in range(self.iters_per_call):
                e, J = self._error_and_jac(q, T)
                if np.linalg.norm(e) < self.tol:
                    break
                we = w * e
                WJ = w[:, None] * J
                damp2 = float(we @ we) + self.sugihara_bias
                Jpinv = self._damped_pinv(WJ, damp2)
                dq_task = -Jpinv @ we
                N = self._null_projector(WJ, In)
                dq_null = (self.k_posture * (self.q_posture - q)
                           + self.k_limit * self._limit_gradient(q))
                dq = self._soft_limit_scale(q, dq_task + N @ dq_null)
                n = np.linalg.norm(dq)
                if n > 0.5:
                    dq *= 0.5 / n
                q = pin.integrate(self.model, q, dq)

            step = q - q_start
            sn = np.linalg.norm(step)
            if sn > self.dq_max_tick:
                q = q_start + step * (self.dq_max_tick / sn)
            sol_q = np.clip(q, self._lo, self._hi)
            if self._smooth > 0.0:
                sol_q = self._smooth * q_start + (1.0 - self._smooth) * sol_q
            if not np.all(np.isfinite(sol_q)):
                raise ValueError("IK 해에 NaN")
        except Exception as e:
            if not safe:
                raise
            self._warn_once(e)
            sol_q = self.history_data.copy()
        self.init_data = sol_q
        self.history_data = sol_q
        return sol_q
