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


def xyzquat_to_mat(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    mat[:3, 3] = np.array([x, y, z])
    return mat


class ArmIK:
    def __init__(
        self,
        urdf_path: str,
        package_dirs: List[str],
        locked_joints: List[str],
        ee_parent_joint: str,
        ee_frame_name: str = "ee",
        tool_pre_rot_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        tool_translation_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        w_pos: float = 20.0,
        w_ori: float = 1.0,
        w_reg: float = 0.01,
        w_smooth: float = 5.0,
        ipopt_max_iter: int = 50,
        ipopt_tol: float = 1e-4,
        collision_pairs_flat: Optional[Sequence[int]] = None,
        enable_collision_check: bool = False,
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

        # ---- TCP(말단공구) 외부파라미터를 ee_parent_joint 아래 frame 으로 등록 ----
        first = xyzrpy_to_mat(0.0, 0.0, 0.0, tool_pre_rot_rpy[0], tool_pre_rot_rpy[1], tool_pre_rot_rpy[2])
        second = xyzrpy_to_mat(tool_translation_xyz[0], tool_translation_xyz[1], tool_translation_xyz[2], 0.0, 0.0, 0.0)
        ee_mat = first @ second
        quat = Rotation.from_matrix(ee_mat[:3, :3]).as_quat()  # x y z w
        local = pin.SE3(pin.Quaternion(quat[3], quat[0], quat[1], quat[2]),
                        np.array(ee_mat[:3, 3]))               # 부모 프레임 기준 TCP
        # ee_parent 가 조인트면 그 조인트 아래, fixed 프레임(예: gripper_flange_joint)
        # 이면 그 프레임의 지지 조인트 아래에 프레임 placement 를 접어 등록.
        if model.existJointName(ee_parent_joint):
            jid = model.getJointId(ee_parent_joint)
            placement = local
        else:
            pf = model.frames[model.getFrameId(ee_parent_joint)]
            jp = getattr(pf, "parentJoint", None)
            jid = int(jp if jp is not None else pf.parent)
            placement = pf.placement * local                  # jMf ∘ local
        # pinocchio 버전별 Frame 시그니처 차이:
        #   신형: Frame(name, parent_joint, parent_frame, placement, type)
        #   구형: Frame(name, parent_joint, placement, type)
        try:
            parent_frame = model.getFrameId(model.names[jid])   # 부모 조인트 프레임
            frame = pin.Frame(ee_frame_name, jid, parent_frame,
                              placement, pin.FrameType.OP_FRAME)
        except Exception:
            frame = pin.Frame(ee_frame_name, jid, placement, pin.FrameType.OP_FRAME)
        model.addFrame(frame)

        self.robot = None
        self.reduced_robot = None
        self.model = model
        self.ee_id = model.getFrameId(ee_frame_name)
        self.enable_collision_check = False
        self.geom_model = None
        self.geometry_data = None
        _ = (package_dirs, collision_pairs_flat, enable_collision_check)

        self._finish_setup(w_pos, w_ori, w_reg, w_smooth, ipopt_max_iter, ipopt_tol)

    # ----------------------------------------------------------------- builders
    def _finish_setup(self, w_pos, w_ori, w_reg, w_smooth, max_iter, tol):
        self.w_pos = float(w_pos)
        self.w_ori = float(w_ori)
        self.w_reg = float(w_reg)
        self.w_smooth = float(w_smooth)
        self.max_iter = int(max_iter)
        self.tol = float(tol)
        
        w = np.array([w_pos] * 3 + [w_ori] * 3, dtype=float)
        self._task_w = w / max(w.max(), 1e-9)
        self._damp = 1e-2                 # DLS 감쇠(λ) — 특이자세 안정화
        # 관절 한계(유한화) + soft clamp 파라미터
        self._lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                            self.model.lowerPositionLimit, -np.pi)
        self._hi = np.where(np.isfinite(self.model.upperPositionLimit),
                            self.model.upperPositionLimit, np.pi)
        self._limit_margin = 0.10         # [rad] 한계 근처 soft 존 폭
        self._k_limit = 0.15              # 여유자유도 한계회피 이득(낮을수록 덜 진동)
        self._smooth = 0.2               
        
        self._q_neutral = pin.neutral(self.model)
        # ee frame 이 addFrame 으로 추가된 뒤의 model 에 맞춰 data 재생성
        # (기존 self.data 는 프레임 추가 전 생성돼 oMf[ee_id] 가 없다)
        self.data = self.model.createData()
        self.init_data = np.zeros(self.model.nq)
        self.history_data = np.zeros(self.model.nq)
        self._fk_data = self.model.createData()

    @classmethod
    def from_appended(
        cls, arm_urdf: str, hand_urdf: str, attach_frame: str, ee_link: str,
        mount_xyz: Sequence[float] = (0.0, 0.0, 0.0), mount_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        locked_joints: Optional[List[str]] = None,
        w_pos: float = 20.0, w_ori: float = 10.0, w_reg: float = 0.01, w_smooth: float = 0.01,
        ipopt_max_iter: int = 50, ipopt_tol: float = 1e-4,
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
        # 활성 조인트 = universe→ee_link 지지 체인의 이동 가능 조인트 − 잠금.
        ee_frame = combined.frames[combined.getFrameId(ee_link)]
        j_ee = getattr(ee_frame, "parentJoint", None)
        if j_ee is None:                         # pin 구버전 호환
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
        
        self.robot = None
        self.reduced_robot = None
        self.model = reduced
        self.data = reduced.createData()
        self.ee_id = reduced.getFrameId(ee_link)
        self.enable_collision_check = False
        self.geom_model = None
        self.geometry_data = None
        self._finish_setup(w_pos, w_ori, w_reg, w_smooth, ipopt_max_iter, ipopt_tol)
        return self

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

    # ------------------------------------------------------------------- solve
    def _error_and_jac(self, q: np.ndarray, T: np.ndarray):
        q = np.asarray(q, dtype=float)
        # 이 q 로 FK 갱신 후 프레임 placement 반영 (oMf 를 최신화) → 오차 계산
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        iMd = self.data.oMf[self.ee_id].actInv(pin.SE3(np.asarray(T, dtype=float)))
        e = pin.log6(iMd).vector
        # LOCAL 프레임 야코비안 + Jlog6 → ∂e/∂q (pinocchio IK 예제 표준형)
        Jf = pin.computeFrameJacobian(self.model, self.data, q, self.ee_id, pin.LOCAL)
        J = -pin.Jlog6(iMd.inverse()) @ Jf
        
        return e, J

    def _limit_gradient(self, q: np.ndarray) -> np.ndarray:
        m = self._limit_margin
        g = np.zeros_like(q)
        low_head = q - self._lo         # 하한까지 여유
        high_head = self._hi - q        # 상한까지 여유
        near_low = low_head < m
        near_high = high_head < m
        g[near_low] += (m - low_head[near_low]) / m
        g[near_high] -= (m - high_head[near_high]) / m
        return g

    def _soft_limit_scale(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        m = self._limit_margin
        out = dq.copy()
        up = dq > 0
        dn = dq < 0
        head = np.where(up, self._hi - q, np.where(dn, q - self._lo, m))
        scale = np.clip(head / m, 0.0, 1.0)
        near = head < m
        out[near] *= scale[near]
        return out

    def converge(self, target_pose: np.ndarray, q0: np.ndarray) -> np.ndarray:
        # 순수 함수: q0 에서 시작해 수렴까지 반복 → 해. 상태(history_data)를 읽지도
        # 쓰지도 않고 출력 EMA 도 적용하지 않는다. solve() 와 solve_robust() 가
        # **둘 다 이걸** 쓴다 — 전역 탐색이 서브클래스의 solve() 를 타면 그 클래스의
        # 평활/rate-limit 이 후보 해에 섞여 들어간다(실측 45mm 오염 사례).
        T = np.asarray(target_pose, dtype=float)
        q = np.array(q0, dtype=float)
        w = self._task_w
        damp2 = self._damp * self._damp
        I6 = np.eye(6)
        In = np.eye(self.model.nq)
        for _ in range(self.max_iter):
            e, J = self._error_and_jac(q, T)
            if np.linalg.norm(e) < self.tol:
                break
            we = w * e
            WJ = w[:, None] * J
            Jpinv = WJ.T @ np.linalg.solve(WJ @ WJ.T + damp2 * I6, I6)   # damped 유사역
            dq_task = -Jpinv @ we
            # 여유자유도로 한계 회피(주태스크 불간섭): N = I - J⁺J
            N = In - Jpinv @ WJ
            dq = dq_task + N @ (self._k_limit * self._limit_gradient(q))
            dq = self._soft_limit_scale(q, dq)           # 한계 쪽 속도 감쇠(soft)
            n = np.linalg.norm(dq)
            if n > 1.0:                                  # 스텝 노름 제한(발산 방지)
                dq *= 1.0 / n
            q = pin.integrate(self.model, q, dq)
        return np.clip(q, self._lo, self._hi)   # soft 로 거의 안 닿지만 안전 clip

    def solve(self, target_pose: np.ndarray, safe: bool = True) -> np.ndarray:
        try:
            sol_q = self.converge(target_pose, self.history_data)
            # 출력 EMA 평활 — 프레임 간 떨림 억제(직전 해와 블렌드)
            if self._smooth > 0.0:
                sol_q = self._smooth * self.history_data + (1.0 - self._smooth) * sol_q
            if not np.all(np.isfinite(sol_q)):
                raise ValueError("IK 해에 NaN")
        except Exception as e:
            if not safe:
                raise
            # 직전 해 유지(발산/NaN 방어). 다만 조용히 삼키면 "IK 가 안 따라온다" 와
            # 구분이 안 되므로 예외 종류별로 한 번은 반드시 알린다.
            self._warn_once(e)
            sol_q = self.history_data.copy()
        self.init_data = sol_q
        self.history_data = sol_q
        return sol_q

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

    def solve_dls(self, target_pose: np.ndarray, iters: int = 10,
                  damp: float = 1e-2, tol: float = 1e-4) -> np.ndarray:
        lo = self.model.lowerPositionLimit
        hi = self.model.upperPositionLimit
        I6 = np.eye(6)
        q = np.array(self.history_data, dtype=float)
        T = np.asarray(target_pose, dtype=float)
        for _ in range(iters):
            e, J = self._error_and_jac(q, T)
            if np.linalg.norm(e) < tol:
                break
            dq = -J.T @ np.linalg.solve(J @ J.T + (damp * damp) * I6, e)
            q = np.clip(pin.integrate(self.model, q, dq), lo, hi)
        if not np.all(np.isfinite(q)):
            q = self.history_data.copy()
        self.init_data = q
        self.history_data = q
        return q

    def fk(self, q: np.ndarray) -> np.ndarray:
        pin.framesForwardKinematics(self.model, self._fk_data, np.asarray(q, dtype=float))
        return self._fk_data.oMf[self.ee_id].homogeneous.copy()

    def frame_pose(self, frame_name: str, q: np.ndarray) -> np.ndarray:
        pin.framesForwardKinematics(self.model, self._fk_data, np.asarray(q, dtype=float))
        return self._fk_data.oMf[self.model.getFrameId(frame_name)].homogeneous.copy()

    def has_frame(self, frame_name: str) -> bool:
        return self.model.existFrame(frame_name)

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
        lo, hi = self._lo, self._hi           # 유한화된 한계(무한 한계도 ±π 로)

        best_q = None
        best_score = np.inf
        # 후보 평가는 converge()(순수 함수) — 서브클래스의 solve()를 타지 않으므로
        # 그 클래스의 평활/rate-limit 이 후보에 섞이지 않고, 상태도 건드리지 않는다.
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
        n_local = (n - 1) // 3 if w_dist > 0.0 else 0     # 탈출 모드에서만 지역 섭동
        for k in range(n_local):
            scale = 0.15 + 0.45 * (k / max(1, n_local - 1))
            yield np.clip(q_ref + scale * span * (rng.random(self.nq) - 0.5), lo, hi)
        for _ in range(max(0, n - 1 - n_local)):
            yield lo + span * rng.random(self.nq)

    def solve_xyzrpy(self, x, y, z, roll, pitch, yaw) -> np.ndarray:
        return self.solve(xyzrpy_to_mat(x, y, z, roll, pitch, yaw))

    def solve_xyzquat(self, x, y, z, qx, qy, qz, qw) -> np.ndarray:
        return self.solve(xyzquat_to_mat(x, y, z, qx, qy, qz, qw))

    def check_self_collision(self, q: np.ndarray) -> bool:
        if self.geom_model is None:
            return False  # 결합(appended) 모델은 충돌 기하 없음
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.geom_model, self.geometry_data)
        return pin.computeCollisions(self.geom_model, self.geometry_data, False)


class DiffArmIK(ArmIK):

    # 텔레옵 스텝 파라미터 (인스턴스에서 덮어쓰기 가능)
    iters_per_call = 100      # 틱당 IK 스텝 수 (내부 완전 수렴 — 지연 없음)
    dp_max = 1.0           
    dtheta_max = 3.15       
    dq_max_tick = 0.5        
    k_posture = 0.0        
    sugihara_bias = 1e-4   
    def _finish_setup(self, *a, **k):
        super()._finish_setup(*a, **k)
        self._smooth = 0.2                      # 출력 EMA(이전 5%만) — 가벼운 떨림 억제
        self.q_posture = self._q_neutral.copy()  # 선호 자세 (기본 중립)

    def _rate_limited_target(self, q: np.ndarray, T_goal: np.ndarray) -> np.ndarray:
        T_cur = self.fk(q)
        T = np.asarray(T_goal, dtype=float).copy()
        # 위치: 스텝 노름 제한
        dp = T[:3, 3] - T_cur[:3, 3]
        n = np.linalg.norm(dp)
        if n > self.dp_max:
            T[:3, 3] = T_cur[:3, 3] + dp * (self.dp_max / n)
        # 자세: 상대회전 각도 제한 (축각 보간)
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
        I6 = np.eye(6)
        In = np.eye(self.model.nq)
        try:
            T = self._rate_limited_target(q, T_goal)
            for _ in range(self.iters_per_call):
                e, J = self._error_and_jac(q, T)
                if np.linalg.norm(e) < self.tol:
                    break
                we = w * e
                WJ = w[:, None] * J
                # Sugihara 오차적응 감쇠 — 오차 클수록(도달불가) 강하게 감쇠
                damp2 = float(we @ we) + self.sugihara_bias
                Jpinv = WJ.T @ np.linalg.inv(WJ @ WJ.T + damp2 * I6)
                dq_task = -Jpinv @ we
                # null-space: 선호자세 + 관절한계 회피 (주태스크 불간섭)
                N = In - Jpinv @ WJ
                dq_null = (self.k_posture * (self.q_posture - q)
                           + self._k_limit * self._limit_gradient(q))
                dq = self._soft_limit_scale(q, dq_task + N @ dq_null)
                n = np.linalg.norm(dq)
                if n > 0.5:                      # 이터레이션당 스텝 제한(발산 방지)
                    dq *= 0.5 / n
                q = pin.integrate(self.model, q, dq)
                
            
            step = q - q_start
            sn = np.linalg.norm(step)
            if sn > self.dq_max_tick:
                q = q_start + step * (self.dq_max_tick / sn)
            sol_q = np.clip(q, self._lo, self._hi)
            if self._smooth > 0.0:               # 출력 EMA(이전 _smooth 만) — 가벼운 떨림 억제
                sol_q = self._smooth * q_start + (1.0 - self._smooth) * sol_q
            if not np.all(np.isfinite(sol_q)):
                raise ValueError("IK 해에 NaN")
        except Exception as e:
            if not safe:
                raise
            # 직전 해 유지(발산/NaN 방어). 다만 조용히 삼키면 "IK 가 안 따라온다" 와
            # 구분이 안 되므로 예외 종류별로 한 번은 반드시 알린다.
            self._warn_once(e)
            sol_q = self.history_data.copy()
        self.init_data = sol_q
        self.history_data = sol_q
        return sol_q
