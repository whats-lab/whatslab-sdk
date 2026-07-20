"""Standalone arm IK solver (ROS-free, pip 전용).

원본 QuestArmTeleop 의 솔버(`ArmIK`) 부분만 분리한 것. pinocchio 의 해석
야코비안(`computeFrameJacobian`+`Jlog6`) + Damped Least-Squares(Gauss-Newton)로
말단(end-effector) 목표 pose 를 받아 관절각을 계산한다.

casadi/IPOPT 를 쓰지 않으므로 pip pinocchio(double 바인딩) 하나면 동작한다
(conda-forge pinocchio 불필요 → 손/팔/리시버/viz 가 단일 pip 스택으로 통일).

의존성: pinocchio, numpy, scipy
"""
from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation


def xyzrpy_to_mat(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """xyz 위치 + RPY(rad, 'xyz' 순서) 회전 -> 4x4 동차변환행렬."""
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    mat[:3, 3] = np.array([x, y, z])
    return mat


def xyzquat_to_mat(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """xyz 위치 + quaternion(x,y,z,w) -> 4x4 동차변환행렬."""
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
        w_ori: float = 2.0,
        w_reg: float = 0.01,
        w_smooth: float = 2.0,
        ipopt_max_iter: int = 50,
        ipopt_tol: float = 1e-4,
        collision_pairs_flat: Optional[Sequence[int]] = None,
        enable_collision_check: bool = False,
    ):
        # ---- 기구학 전용 모델 로드(메쉬 불필요) & 잠긴 관절 제거 ----
        # buildModelFromUrdf 는 링크/조인트만 읽어 메쉬(package://)를 요구하지 않는다
        # → 내장 URDF(메쉬 없음)로도 동작. (RobotWrapper 는 geom 로드로 메쉬를 강제해 불가)
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
        # 기구학 전용 빌드라 충돌 기하 없음. self-collision 검사는 메쉬가 필요해
        # 내장(메쉬 없음) 구성에선 미지원 — 레거시 시그니처만 수용.
        self.enable_collision_check = False
        self.geom_model = None
        self.geometry_data = None
        _ = (package_dirs, collision_pairs_flat, enable_collision_check)

        self._finish_setup(w_pos, w_ori, w_reg, w_smooth, ipopt_max_iter, ipopt_tol)

    # ----------------------------------------------------------------- builders
    def _finish_setup(self, w_pos, w_ori, w_reg, w_smooth, max_iter, tol):
        """수치 IK(DLS/Gauss-Newton) 파라미터 저장 — casadi/IPOPT 불필요.

        비용은 casadi 버전과 동일한 항: 가중 pose 오차 + 중립자세 정규화(w_reg) +
        직전해 평활화(w_smooth). 야코비안은 pinocchio 해석식을 쓴다.
        (인자명 max_iter/tol 은 과거 ipopt_* 위치에 매핑 — 시그니처 호환)
        """
        self.w_pos = float(w_pos)
        self.w_ori = float(w_ori)
        self.w_reg = float(w_reg)
        self.w_smooth = float(w_smooth)
        self.max_iter = int(max_iter)
        self.tol = float(tol)
        # 6D task 상대 우선순위(위치 3 + 자세 3) — 오차/야코비안 행 스케일링.
        # 최댓값으로 정규화(≤1) → damp 가 상대적 의미를 갖게 함. so101 처럼 w_ori<<w_pos
        # 이면 자연히 위치 우선(자세는 여유자유도로 흡수).
        w = np.array([w_pos] * 3 + [w_ori] * 3, dtype=float)
        self._task_w = w / max(w.max(), 1e-9)
        self._damp = 1e-2                 # DLS 감쇠(λ) — 특이자세 안정화
        # 관절 한계(유한화) + soft clamp 파라미터
        self._lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                            self.model.lowerPositionLimit, -np.pi)
        self._hi = np.where(np.isfinite(self.model.upperPositionLimit),
                            self.model.upperPositionLimit, np.pi)
        self._limit_margin = 0.20         # [rad] 한계 근처 soft 존 폭
        self._k_limit = 0.15              # 여유자유도 한계회피 이득(낮을수록 덜 진동)
        self._smooth = 0.5                # 출력 EMA 평활 [0=off..1). 떨림 억제(약간 지연)
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
        """pin.appendModel 로 팔 끝(attach_frame)에 손을 붙이고, EE=손목/베이스
        링크(ee_link)를 두는 결합 솔버. 기구학 전용(메쉬/충돌 없음).

        활성(IK 대상) 조인트 = universe→ee_link 지지 체인의 이동 가능 조인트 −
        locked_joints. 체인 밖(손가락 등)은 손 리타게팅 소관이라 잠근다. 손목에
        구동 관절이 있고(orca 카펄) 손 리타게팅이 고정으로 두면, 그 관절은 이
        체인에 포함되어 팔 IK 의 여분 DOF 로 쓰인다.

        mount_xyz/rpy: 팔 attach_frame → 손 베이스 장착 변환(기계 도면값). 기본 0.
        """
        self = cls.__new__(cls)
        m_arm = pin.buildModelFromUrdf(arm_urdf)
        m_hand = pin.buildModelFromUrdf(hand_urdf)
        # pinocchio 는 appendModel 시 두 모델의 프레임 이름 충돌을 거부한다.
        # 겹치는 손 프레임을 전부 개명한다 — 버전마다 자동 생성 프레임이 달라
        # (pin 2.7: universe+root_joint, pin 3.x: universe) 하드코딩은 취약.
        # ee_link 는 append 후 이름으로 조회하므로 개명 제외.
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

        # EE 프레임 로컬 축 보정. 레거시 기본값 (0,-π/2,π/2) 은 nero+orca 실측
        # 정렬 — RobotModel 경로는 rig config(attach/axis_align)로 방향을 다루므로
        # ee_local_rpy=(0,0,0) 을 전달한다.
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
        """현재 실제 관절각으로 초기값을 동기화 (연속적인 해를 얻기 위함)."""
        q = np.array(q_current, dtype=float)
        if q.shape[0] == self.nq:
            self.init_data = q
            self.history_data = q

    # ------------------------------------------------------------------- solve
    def _error_and_jac(self, q: np.ndarray, T: np.ndarray):
        """말단 pose 오차 e=log6(oMf⁻¹·T_target) 와 해석 야코비안 J=∂e/∂q.

        casadi 자동미분과 동일한 오차 정의. J 는 pinocchio 프레임 야코비안(LOCAL)에
        Jlog6 를 곱한 표준형(pinocchio inverse-kinematics 예제와 동일).
        """
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
        """한계 근처에서 중앙으로 미는 그래디언트(soft). 여유 존 밖은 0.

        하한 근처 → 양수(q 증가=이탈), 상한 근처 → 음수. 크기는 0→1 로 램프.
        """
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
        """한계 '쪽으로' 가는 성분을 여유가 줄수록 감쇠(→0 at 한계). hard clip 대체."""
        m = self._limit_margin
        out = dq.copy()
        up = dq > 0
        dn = dq < 0
        head = np.where(up, self._hi - q, np.where(dn, q - self._lo, m))
        scale = np.clip(head / m, 0.0, 1.0)
        near = head < m
        out[near] *= scale[near]
        return out

    def solve(self, target_pose: np.ndarray, safe: bool = True) -> np.ndarray:
        """4x4 목표 pose -> 관절각(rad). 가중 최소노름 DLS + soft 관절한계.

        dq = -(WJ)⁺(W·e) + N·(k·∇limit). 주태스크는 damped 최소노름(위치/자세 우선순위
        W), 여유자유도(null-space N)로 한계에서 서서히 밀어냄(∇limit). 한계 '쪽' 속도는
        여유가 줄수록 감쇠(soft) → hard clip 없이 부드럽게 정지. warm-start 시간 평활.
        safe=True: 수렴 실패/NaN 시 직전 해 반환(라이브 루프 보호).
        """
        T = np.asarray(target_pose, dtype=float)
        q = np.array(self.history_data, dtype=float)
        w = self._task_w
        damp2 = self._damp * self._damp
        I6 = np.eye(6)
        In = np.eye(self.model.nq)
        try:
            for _ in range(self.max_iter):
                e, J = self._error_and_jac(q, T)
                if np.linalg.norm(e) < self.tol:
                    break
                we = w * e
                WJ = w[:, None] * J
                Jpinv = WJ.T @ np.linalg.inv(WJ @ WJ.T + damp2 * I6)  # damped 유사역
                dq_task = -Jpinv @ we
                # 여유자유도로 한계 회피(주태스크 불간섭): N = I - J⁺J
                N = In - Jpinv @ WJ
                dq = dq_task + N @ (self._k_limit * self._limit_gradient(q))
                dq = self._soft_limit_scale(q, dq)           # 한계 쪽 속도 감쇠(soft)
                n = np.linalg.norm(dq)
                if n > 1.0:                                  # 스텝 노름 제한(발산 방지)
                    dq *= 1.0 / n
                q = pin.integrate(self.model, q, dq)
            # soft 로 거의 안 닿지만 마지막 안전 clip(수치 오차 방지)
            sol_q = np.clip(q, self._lo, self._hi)
            # 출력 EMA 평활 — 프레임 간 떨림 억제(직전 해와 블렌드)
            if self._smooth > 0.0:
                sol_q = self._smooth * self.history_data + (1.0 - self._smooth) * sol_q
            if not np.all(np.isfinite(sol_q)):
                raise ValueError("IK 해에 NaN")
        except Exception:
            if not safe:
                raise
            sol_q = self.history_data.copy()
        self.init_data = sol_q
        self.history_data = sol_q
        return sol_q

    def solve_dls(self, target_pose: np.ndarray, iters: int = 10,
                  damp: float = 1e-2, tol: float = 1e-4) -> np.ndarray:
        """최소노름 Damped Least-Squares(Gauss-Newton) IK — 가장 가벼운 추종용.

        dq = -Jᵀ(JJᵀ+λ²I)⁻¹·e 스텝(min-norm → 자연히 '관절 최소 이동'). 관절한계 clamp.
        가중/정규화가 필요하면 solve() 사용. cold-start 정밀해는 solve_robust().
        """
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
        """관절각 q -> ee frame 의 4x4 동차변환행렬 (정기구학)."""
        pin.framesForwardKinematics(self.model, self._fk_data, np.asarray(q, dtype=float))
        return self._fk_data.oMf[self.ee_id].homogeneous.copy()

    def frame_pose(self, frame_name: str, q: np.ndarray) -> np.ndarray:
        """임의 프레임의 4x4 pose (FK). 결합 모델에서 Orca 베이스 등 조회용."""
        pin.framesForwardKinematics(self.model, self._fk_data, np.asarray(q, dtype=float))
        return self._fk_data.oMf[self.model.getFrameId(frame_name)].homogeneous.copy()

    def has_frame(self, frame_name: str) -> bool:
        return self.model.existFrame(frame_name)

    def pose_error(self, q: np.ndarray, target_pose: np.ndarray) -> tuple:
        """(위치오차[m], 자세오차[rad]) 반환 — 해의 실제 정확도 평가용."""
        T = self.fk(q)
        pos_err = float(np.linalg.norm(T[:3, 3] - target_pose[:3, 3]))
        R = T[:3, :3].T @ target_pose[:3, :3]
        ori_err = float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
        return pos_err, ori_err

    def solve_robust(
        self,
        target_pose: np.ndarray,
        restarts: int = 12,
        pos_tol: float = 1e-3,
        ori_tol: float = 1e-2,
        seed: int = 0,
    ) -> np.ndarray:
        """여러 초기값으로 재시작하며 실제 pose 오차가 가장 작은 해를 고른다.

        국소 최적화기가 cold-start 에서 국소최소에 빠지는 것을 방지.
        pos_tol[m]·ori_tol[rad] 를 모두 만족하면 조기 종료한다.
        한 번 호출하고 나면 그 해가 warm-start 로 남으므로, 이어지는 연속 solve()
        는 이 해 근처에서 빠르게 수렴한다.
        """
        rng = np.random.default_rng(seed)
        lo = self.model.lowerPositionLimit
        hi = self.model.upperPositionLimit

        best_q = None
        best_score = np.inf
        # 1번째 시도는 직전해(warm), 이후는 관절범위 내 랜덤
        candidates = [self.history_data.copy()]
        candidates += [lo + (hi - lo) * rng.random(self.nq) for _ in range(max(0, restarts - 1))]

        for q0 in candidates:
            self.init_data = q0
            self.history_data = q0
            try:
                q = ArmIK.solve(self, target_pose)   # 후보 평가는 full-convergence(diff 백엔드도)
            except Exception:
                continue
            pe, oe = self.pose_error(q, target_pose)
            score = pe + 0.1 * oe
            if score < best_score:
                best_score, best_q = score, q
            if pe <= pos_tol and oe <= ori_tol:
                break

        if best_q is None:
            raise RuntimeError("IK 가 어떤 초기값에서도 해를 찾지 못했습니다.")
        self.init_data = best_q
        self.history_data = best_q
        return best_q

    def solve_xyzrpy(self, x, y, z, roll, pitch, yaw) -> np.ndarray:
        """xyz + RPY(rad) -> 7축 관절각."""
        return self.solve(xyzrpy_to_mat(x, y, z, roll, pitch, yaw))

    def solve_xyzquat(self, x, y, z, qx, qy, qz, qw) -> np.ndarray:
        """xyz + quaternion(x,y,z,w) -> 7축 관절각."""
        return self.solve(xyzquat_to_mat(x, y, z, qx, qy, qz, qw))

    def check_self_collision(self, q: np.ndarray) -> bool:
        if self.geom_model is None:
            return False  # 결합(appended) 모델은 충돌 기하 없음
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.geom_model, self.geometry_data)
        return pin.computeCollisions(self.geom_model, self.geometry_data, False)


class DiffArmIK(ArmIK):
    """미분 IK(differential IK) 백엔드 — 텔레옵용 안정화 조합.

    ArmIK("dls")가 매 프레임 수렴까지 반복해 인접 프레임에서 다른 국소해
    (elbow flip)로 튈 수 있는 반면, 이 백엔드는:
      · 틱당 소수 스텝만 밟아 해가 항상 현재 자세의 연속 (CLIK)
      · 목표 rate-limit: 목표 pose 를 현재 EE 에서 최대 이동량만큼만 접근시켜
        입력 점프/노이즈에 강함 (출력 EMA 불필요 → 제거)
      · Sugihara 오차적응 감쇠: λ² = ‖W·e‖² + bias — 도달 불가 목표에서도
        발산하지 않음 (Sugihara 2011 LM)
      · null-space 자세 태스크: 여유자유도를 선호자세(q_posture)로 끌어
        elbow 방황 방지 (+기존 관절한계 회피)

    인터페이스는 ArmIK 와 동일 — solve(T)->q. 튜닝은 속성으로:
      iters_per_call, dp_max, dtheta_max, k_posture, sugihara_bias, q_posture
    """

    # 텔레옵 스텝 파라미터 (인스턴스에서 덮어쓰기 가능)
    iters_per_call = 2       # 틱당 IK 스텝 수
    dp_max = 0.05            # [m]   틱당 목표 위치 최대 접근량
    dtheta_max = 0.25        # [rad] 틱당 목표 자세 최대 접근량
    k_posture = 0.05         # null-space 선호자세 이득
    sugihara_bias = 1e-4     # 감쇠 바이어스 (0 방지)

    def _finish_setup(self, *a, **k):
        super()._finish_setup(*a, **k)
        self._smooth = 0.0                       # 출력 EMA 불필요 (rate-limit 로 대체)
        self.q_posture = self._q_neutral.copy()  # 선호 자세 (기본 중립)

    def _rate_limited_target(self, q: np.ndarray, T_goal: np.ndarray) -> np.ndarray:
        """현재 EE pose 에서 T_goal 방향으로 (dp_max, dtheta_max) 만큼만 이동한 목표."""
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
        q = np.array(self.history_data, dtype=float)
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
                if n > 0.5:                      # 틱당 관절 스텝 제한
                    dq *= 0.5 / n
                q = pin.integrate(self.model, q, dq)
            sol_q = np.clip(q, self._lo, self._hi)
            if not np.all(np.isfinite(sol_q)):
                raise ValueError("IK 해에 NaN")
        except Exception:
            if not safe:
                raise
            sol_q = self.history_data.copy()
        self.init_data = sol_q
        self.history_data = sol_q
        return sol_q
