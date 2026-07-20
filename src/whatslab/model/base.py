"""TeleopModel — 사용자 대면 최상위 API (4단계 파이프라인, 항상 양손 처리).
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Dict, Optional

import numpy as np

from whatslab.core.interfaces import HandController
from whatslab.core.types import Pose
from whatslab.robot import RobotModel
from .calibration import ArmCalibration
from .ik import RobotArmIK


class TeleopModel(ABC):
    """텔레옵 최상위 — get_data → calib → solve → get_q (항상 양손).

    사용 예:
        class MyModel(TeleopModel):
            def __init__(self, robot):
                self.arm_source  = QuestControllerReceiver()
                self.hand_source = GloveHumanHandReceiver()
                super().__init__(robot)          # 손 리타게팅 config 는 rig.hand.retarget 에서 유도
            def _get_raw_target(self):                         # 팔 필드만 재정의(양손)
                out = {}
                for s in self.SIDES:
                    c = self.arm_source.get(s).controller
                    out[s] = Pose(c.pos, c.quat) if c is not None else None
                return out                                   # 팔=컨트롤러(양손), None=IK 스킵
    """

    arm_source = None
    hand_source = None
    SIDES = ("left", "right")       # 항상 양쪽 처리 — 특별한 side 를 고정하지 않는다.

    def __init__(self, robot):
        self.robots: Dict[str, RobotModel] = self._as_side_map(robot)

        # 편의: 유일 rig 면 self.robot 로도 접근(서로 다른 양손이면 None → self.robots).
        uniq = {id(r): r for r in self.robots.values()}
        self.robot = next(iter(uniq.values())) if len(uniq) == 1 else None

        # 선택: 관절 리밋/e-stop 필터. 소비처가 SafetyFilter(limits, dt) 를 주입하면
        # get_q 출력이 이를 통과한다(리밋 클램프/홀드). None 이면 미적용.
        self.safety = None

        self.target: Dict[str, Optional[np.ndarray]] = {}   # side → 팔 EE 목표 4x4 | None
        self.q: Dict[str, Dict[str, float]] = {}

        self.ik: Dict[str, RobotArmIK] = {}
        self.retarget: Dict[str, HandController] = {}
        self.calib: Dict[str, ArmCalibration] = {}

        for s, r in self.robots.items():
            cfg = r.rig.hand.retarget if r.rig.hand is not None else None
            if r.has_arm:
                self.ik[s] = RobotArmIK(r)
                self.calib[s] = ArmCalibration(
                    reach_max=r.rig.solver.reach_max,
                    input_reach=r.rig.calibration.input_reach)
            if r.has_hand and cfg:
                self.retarget[s] = r.make_hand_controller(cfg, s)

    def _as_side_map(self, robot) -> Dict[str, RobotModel]:
        """robot → {side: RobotModel}. **단일 rig = 양쪽(left/right) 모두 그 rig**,
        [l, r]=left/right 순서, dict=그대로."""
        def _load(r):
            if isinstance(r, str):
                return RobotModel.from_yaml(r)
            return r
        if robot is None:
            return {}
        if isinstance(robot, dict):
            return {s: _load(r) for s, r in robot.items()}
        if isinstance(robot, (list, tuple)):
            return {self.SIDES[i]: _load(r) for i, r in enumerate(robot) if r is not None}
        r = _load(robot)
        return {s: r for s in self.SIDES}        # 단일 rig → 양쪽 다 동일 rig

    # ------------------------------------------------------------- lifecycle
    @property
    def _receivers(self) -> list:
        out, seen = [], set()
        for r in (self.arm_source, self.hand_source):
            if r is not None and id(r) not in seen:
                seen.add(id(r))
                out.append(r)
        return out

    def start(self) -> None:
        for r in self._receivers:
            r.start()

    def stop(self) -> None:
        for r in self._receivers:
            r.stop()

    # ================================================================ pipeline
    @abstractmethod
    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        """side 별 팔 EE 원시 자세 `Pose`(pos+quat) — 양손 반환(**서브클래스 필수 구현**).
        어느 소스/크로스핸드 조합을 팔 목표로 쓸지 정한다. 특정 side 를 `None` 으로
        주면 그 side 는 IK 를 풀지 않는다. 팔 없는 모델은 `{s: None for s in self.SIDES}`."""
        ...

    def get_data(self) -> Dict[str, dict]:
        """양손 소스 수집 + 역할 결정 → {side: data}. 팔 자세(`arm_pose`)는
        `_get_raw_target` 가 준 프레임 그대로 통과시킨다(head-relative 등 프레임 선택은
        그 훅 소관). `tracked` = 그 side 에 유효 입력(팔 목표 또는 손가락)이 있나."""
        poses = self._get_raw_target()                       # {side: Pose|None}
        out: Dict[str, dict] = {}
        for s in self.SIDES:
            arm_s = self.arm_source.get(s) if self.arm_source else None
            hand_s = self.hand_source.get(s) if self.hand_source else None
            arm_pose = poses.get(s)
            out[s] = {
                "arm_pose": arm_pose,                       # _get_raw_target 프레임 그대로
                "fingers": hand_s,
                "q": self._joint_q(arm_s, hand_s),          # 직접-q 우회(손 우선)
                "tracked": arm_pose is not None or self._has_fingers(hand_s),
            }
        return out

    @staticmethod
    def _joint_q(arm_s, hand_s):
        """직접-q 우회값 — 손 소스 우선, 없으면 팔 소스, 둘 다 없으면 None."""
        if hand_s is not None and hand_s.joint_q is not None:
            return hand_s.joint_q
        if arm_s is not None and arm_s.joint_q is not None:
            return arm_s.joint_q
        return None

    def _solve_side(self, side: str, data: dict) -> Dict[str, float]:
        """한 side data → q. joint_q 우회가 있으면 그대로. 팔 목표(arm_target)나
        손가락 입력이 없는 컴포넌트는 계산도 출력도 하지 않고 q 에서 **생략**한다."""
        if data.get("q") is not None:
            return dict(data["q"])
        q: Dict[str, float] = {}
        ik = self.ik.get(side)
        T = data.get("arm_target")
        if ik is not None and T is not None:                    # 팔 목표 없으면 IK 생략
            q_arm = np.asarray(ik.solve(T), dtype=float)
            q.update(zip(ik.joint_names, (float(v) for v in q_arm)))
        retarget = self.retarget.get(side)
        fingers = data.get("fingers")
        if retarget is not None and self._has_fingers(fingers):  # 손가락 없으면 리타게팅 생략
            cmd = retarget.compute(fingers)
            q.update(zip(cmd.joint_names, (float(v) for v in cmd.joint_angles)))
        return q

    @staticmethod
    def _has_fingers(fingers) -> bool:
        """리타게팅할 유효 손가락 입력이 있나 — InputSample 의 hand 가 추적 중일 때만."""
        return (fingers is not None and fingers.hand is not None
                and fingers.hand.tracked)

    def solve(self, data: Dict[str, dict]) -> Dict[str, Dict[str, float]]:
        """{side: (캘리브된) data} → {side: q}. 항상 양쪽 처리."""
        return {s: self._solve_side(s, data[s]) for s in self.SIDES}

    def _apply_calib(self, data: Dict[str, dict]) -> Dict[str, dict]:
        """side 별 calib.apply(yaw 정렬 + reach 스케일) → arm_target 채움 + self.target 노출."""
        for s in self.SIDES:
            calib = self.calib.get(s)
            if calib is not None:
                data[s] = calib.apply(data[s])
            self.target[s] = data[s].get("arm_target")
        return data

    def get_q(self) -> Dict[str, Dict[str, float]]:
        """get_data → _apply_calib → solve → {side: {joint: 값}}. 매 호출 계산."""
        data = self._apply_calib(self.get_data())
        q = self.solve(data)
        if self.safety is not None:              # 선택: 관절 리밋/e-stop 필터
            q = {s: self.safety.step(v) for s, v in q.items()}
        self.q = q                          # 최신 q 노출(양손)
        return q

    # ------------------------------------------------------------- calibrate
    def set_reach(self, input_reach: float) -> Dict[str, bool]:
        """측정 없이 reach(input_reach)를 양쪽 calib 에 직접 설정 → {side: 성공여부}."""
        out: Dict[str, bool] = {}
        for s in self.SIDES:
            calib = self.calib.get(s)
            if calib is not None:
                calib.set_reach(float(input_reach))
            out[s] = calib is not None
        return out

    def calibrate_yaw(self) -> Dict[str, bool]:
        """yaw 정렬 스냅샷(즉시) — 양쪽 calib 모델에 위임 → {side: 성공여부}.
        (도달반경 캘리브는 calibrate_reach.)"""
        data = self.get_data()
        out: Dict[str, bool] = {}
        for s in self.SIDES:
            calib = self.calib.get(s)
            out[s] = bool(calib.capture(data[s])) if calib is not None else False
        return out

    def calibrate_reach(self, duration: float = 8.0, rate_hz: float = 60.0,
                        persist: bool = False) -> Dict[str, float]:
        """duration 초 동안 양쪽 get_data 의 arm_pose 를 폴링해 side 별 최대 도달반경
        측정 → 해당 side calib 에 reach 등록(블로킹) → {side: r_max}. 측정 필드는
        get_data 가 결정한다. persist=True 면 side rig yaml 의 calibration.input_reach
        에도 저장(다음 세션 생성 시 자동 로드)."""
        r_max: Dict[str, float] = {s: 0.0 for s in self.SIDES}
        period, t_end = 1.0 / rate_hz, time.monotonic() + duration
        while time.monotonic() < t_end:
            data = self.get_data()
            for s in self.SIDES:
                pose = data[s].get("arm_pose")
                if pose is not None:
                    r_max[s] = max(r_max[s], float(np.linalg.norm(np.asarray(pose.pos, dtype=float))))
            time.sleep(period)
        for s in self.SIDES:
            calib = self.calib.get(s)
            if calib is not None and r_max[s] > 0.0:
                calib.set_reach(r_max[s])
                if persist:
                    from whatslab.robot import save_calibration
                    robot = self.robots.get(s)
                    if robot is not None:
                        save_calibration(robot.rig, r_max[s])
        return r_max

    # ------------------------------------------------------------- feedback
    def send_feedback(self, data) -> None:
        """햅틱 등 역방향 피드백 — 기본 no-op(서브클래스 오버라이드)."""
