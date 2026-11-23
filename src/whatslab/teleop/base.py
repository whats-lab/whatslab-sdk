from __future__ import annotations

import logging
import os
import threading
import time
from abc import ABC, abstractmethod
from typing import Dict, Optional, Tuple

import numpy as np

from whatslab.core.constants import SIDES
from whatslab.core.types import InputSample, Pose
from whatslab.robot import RobotModel, save_calibration
from whatslab.robot.config import RigConfig, load_rig

from .side import SideModel, _has_fingers

logger = logging.getLogger(__name__)


class TeleopModel(ABC):

    arm_source = None
    hand_source = None

    def __init__(self, robot):
        robots = self._as_side_map(robot)
        self.sides: Dict[str, SideModel] = {
            s: SideModel.build(s, robots.get(s))
            for s in (*SIDES, *(k for k in robots if k not in SIDES))}

        have = [v.robot for v in self.sides.values() if v.robot is not None]
        rigs = {id(getattr(r, "rig", r)) for r in have}
        self.robot = have[0] if len(rigs) == 1 else None

        self._safety = None
        self._t_prev = None
        # 틱 샘플은 스레드별 — 캘리브 스레드와 메인 루프가 동시에 get_data() 를 부른다.
        self._tick = threading.local()
        # 캘리브(capture·reseed)가 한 틱의 보정 적용·IK·안전필터 중간에 끼어들지 않게 한다.
        self._step_lock = threading.Lock()

    @staticmethod
    def _as_rig(r):
        if isinstance(r, RigConfig):
            return r
        if isinstance(r, (str, os.PathLike)):
            return load_rig(os.fspath(r))
        return None

    def _as_side_map(self, robot) -> Dict[str, RobotModel]:
        def _load(r):
            rig = self._as_rig(r)
            return RobotModel(rig) if rig is not None else r
        if robot is None:
            return {}
        if isinstance(robot, dict):
            return {s: _load(r) for s, r in robot.items()}
        if isinstance(robot, (list, tuple)):
            return {SIDES[i]: _load(r) for i, r in enumerate(robot) if r is not None}
        rig = self._as_rig(robot)
        if rig is not None:
            return {s: RobotModel(rig) for s in SIDES}
        logger.warning(
            "%s 인스턴스를 주면 양쪽 side 가 그 유상태 솔버를 공유한다 — side 가"
            " 서로를 밀어낸다(실측 2.6 → 347mm). rig 경로/RigConfig 를 주거나"
            " {side: model} 로 따로 주라.", type(robot).__name__)
        return {s: robot for s in SIDES}

    @property
    def _receivers(self) -> list:
        out, seen = [], set()
        for r in (self.arm_source, self.hand_source):
            if r is not None and id(r) not in seen:
                seen.add(id(r))
                out.append(r)
        return out

    def start(self) -> None:
        """이 모델의 수신기(arm_source·hand_source)를 시작한다.

        수신기 start()/stop() 은 여러 번 불러도 한 번만 동작해야 한다. 하나가 실패하면 그 앞에서
        시작한(이미 돌던 것 포함) 수신기를 멈추고 예외를 올린다. 수신기 인스턴스를 두 모델이 함께
        쓰는 것은 지원하지 않는다 — 한 모델의 stop() 이 다른 모델의 수신기도 멈춘다.
        """
        started = []
        try:
            for r in self._receivers:
                r.start()
                started.append(r)
        except BaseException:
            for r in reversed(started):
                r.stop()
            raise

    def stop(self) -> None:
        """이 모델의 수신기를 멈춘다."""
        for r in self._receivers:
            r.stop()

    @abstractmethod
    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        ...

    def _read_samples(self) -> Dict[str, Tuple[Optional[InputSample], Optional[InputSample]]]:
        """side 별 (팔 샘플, 손 샘플). 팔·손 소스가 같은 수신기면 한 번만 읽는다."""
        out = {}
        for s in SIDES:
            arm_s = self.arm_source.get(s) if self.arm_source else None
            if self.hand_source is self.arm_source:
                hand_s = arm_s
            else:
                hand_s = self.hand_source.get(s) if self.hand_source else None
            out[s] = (arm_s, hand_s)
        return out

    def samples(self) -> Dict[str, Tuple[Optional[InputSample], Optional[InputSample]]]:
        """이번 틱의 (팔, 손) 샘플.

        `_get_raw_target()` 은 수신기를 직접 읽지 말고 이것을 쓴다 — 한 틱에 한 번 읽어 tracked
        판정·시각이 팔과 손에서 같다. `get_data()` 밖에서 부르면 새로 읽는다.
        """
        samples = getattr(self._tick, "samples", None)
        return samples if samples is not None else self._read_samples()

    def get_data(self) -> Dict[str, dict]:
        return self._collect(publish=True)

    def _collect(self, publish: bool) -> Dict[str, dict]:
        """이번 틱 입력. publish 면 side 의 raw_target 도 갱신한다(캘리브 스레드는 안 한다).

        캘리브는 get_data() 가 아니라 이것을 부른다 — 입력 변환은 get_data() 오버라이드가 아니라
        `_get_raw_target()` 에 두어야 메인 루프와 캘리브에 똑같이 적용된다.
        """
        samples = self._read_samples()
        self._tick.samples = samples
        try:
            poses = self._get_raw_target()
        finally:
            self._tick.samples = None
        out: Dict[str, dict] = {}
        for s in SIDES:
            arm_s, hand_s = samples[s]
            arm_pose = poses.get(s)
            if publish:
                self.sides[s].raw_target = arm_pose
            out[s] = {
                "arm_pose": arm_pose,
                "fingers": hand_s,
                "q": self._joint_q(arm_s, hand_s),
                "tracked": arm_pose is not None or _has_fingers(hand_s),
            }
        return out

    @staticmethod
    def _joint_q(arm_s, hand_s):
        if hand_s is not None and hand_s.joint_q is not None:
            return hand_s.joint_q
        if arm_s is not None and arm_s.joint_q is not None:
            return arm_s.joint_q
        return None

    def solve(self, data: Dict[str, dict]) -> Dict[str, Dict[str, float]]:
        return {s: self.sides[s].solve(data[s]) for s in SIDES}

    def _apply_calib(self, data: Dict[str, dict]) -> Dict[str, dict]:
        for s in SIDES:
            data[s] = self.sides[s].apply_calib(data[s])
        return data

    @property
    def safety(self):
        return self._safety

    @safety.setter
    def safety(self, f) -> None:
        self._safety = f
        for m in self.sides.values():
            m.safety = None

    def get_q(self) -> Dict[str, Dict[str, float]]:
        with self._step_lock:  # 읽기도 잠금 안 — 캘리브가 읽기와 적용 사이에 끼지 않게
            return self._step(self.get_data())

    def _step(self, data: Dict[str, dict]) -> Dict[str, Dict[str, float]]:
        data = self._apply_calib(data)
        q = self.solve(data)
        if self._safety is not None:
            now = time.monotonic()
            dt = None if self._t_prev is None else now - self._t_prev
            self._t_prev = now
            for s, v in q.items():
                m = self.sides[s]
                q[s] = m.filter(v, dt, self._safety)
                m.sync_ik(q[s])
        for s, v in q.items():
            self.sides[s].q = v
        return q

    @property
    def q(self) -> Dict[str, Dict[str, float]]:
        return {s: m.q for s, m in self.sides.items()}

    def set_reach(self, input_reach: float) -> Dict[str, bool]:
        out: Dict[str, bool] = {}
        with self._step_lock:
            for s in SIDES:
                calib = self.sides[s].calib
                if calib is not None:
                    calib.set_reach(float(input_reach))
                out[s] = calib is not None
        return out

    def calibrate_yaw(self) -> Dict[str, bool]:
        out: Dict[str, bool] = {}
        data = self._collect(publish=False)
        with self._step_lock:
            for s in SIDES:
                m = self.sides[s]
                ok = bool(m.calib.capture(data[s])) if m.calib is not None else False
                out[s] = ok
                if ok:
                    m.reseed()
        return out

    def calibrate_reach(
        self, duration: float = 8.0, rate_hz: float = 60.0, persist: bool = False
    ) -> Dict[str, float]:
        r_max: Dict[str, float] = {s: 0.0 for s in SIDES}
        period, t_end = 1.0 / rate_hz, time.monotonic() + duration
        while time.monotonic() < t_end:
            data = self._collect(publish=False)
            for s in SIDES:
                pose = data[s].get("arm_pose")
                if pose is not None:
                    r_max[s] = max(
                        r_max[s], float(np.linalg.norm(np.asarray(pose.pos, dtype=float)))
                    )
            time.sleep(period)
        for s in SIDES:
            m = self.sides[s]
            if m.calib is None or r_max[s] <= 0.0:
                continue
            with self._step_lock:
                m.calib.set_reach(r_max[s])
                m.reseed()
            if persist and getattr(m.robot, "rig", None) is not None:
                save_calibration(m.robot.rig, r_max[s])
        return r_max

    def send_feedback(self, data) -> None:
        pass

