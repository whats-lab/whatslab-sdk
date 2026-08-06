from __future__ import annotations

import math
from typing import Dict, List, Optional

_ORCA_LEVELS = {"thumb": ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]}
_DEFAULT_LEVELS = ["{f}_abd", "{f}_mcp", "{f}_pip"]
_FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

ORCA_SIGN: Dict[str, float] = {}


def orca_joint_map(side: str = "right") -> Dict[str, str]:
    from whatslab.teleop.hand.hand_configs import OrcaHandConfig
    cfg = OrcaHandConfig()
    out: Dict[str, str] = {}
    for finger, chain in zip(_FINGER_ORDER, cfg._get_fingers(side)):
        ids = _ORCA_LEVELS.get(finger) or [s.format(f=finger) for s in _DEFAULT_LEVELS]
        for k, oid in enumerate(ids):
            out[f"{chain.links[k + 1]}_to_{chain.links[k]}"] = oid
    return out


class OrcaHandSender:

    def __init__(self, side: str = "right", wrist_joint: Optional[str] = None,
                 model_version: str = "v2"):
        self.side = side
        self.wrist_joint = wrist_joint
        self.model_version = model_version
        self.hand = None
        self._map = orca_joint_map(side)

    def connect(self) -> str:
        from orca_core import OrcaHand
        self.hand = OrcaHand(model_name=f"orcahand_{self.side}",
                             model_version=self.model_version)
        ok, msg = self.hand.connect()
        if not ok:
            self.hand = None
            raise RuntimeError(f"orca 연결 실패: {msg}")
        self.hand.enable_torque()
        return msg

    def close(self) -> None:
        if self.hand is not None:
            try:
                self.hand.disable_torque()
            finally:
                self.hand.disconnect()
                self.hand = None

    @property
    def connected(self) -> bool:
        return self.hand is not None

    def send(self, q: Dict[str, float]) -> None:
        if self.hand is None:
            return
        pose: Dict[str, float] = {}
        for jn, oid in self._map.items():
            if jn in q:
                pose[oid] = math.degrees(q[jn]) * ORCA_SIGN.get(oid, 1.0)
        if self.wrist_joint and self.wrist_joint in q:
            pose["wrist"] = math.degrees(q[self.wrist_joint]) * ORCA_SIGN.get("wrist", 1.0)
        if pose:
            self.hand.set_joint_positions(pose, num_steps=1)


class AgxArmSender:

    def __init__(self, joint_names: List[str], channel: str = "can0",
                 speed_percent: int = 20):
        self.nero_joints = [n for n in joint_names if n.startswith("joint")]
        self.channel = channel
        self.speed_percent = int(speed_percent)
        self.robot = None

    def connect(self) -> str:
        import time
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
        cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT,
                                    interface="socketcan", channel=self.channel)
        robot = AgxArmFactory.create_arm(cfg)
        robot.connect()
        t_end = time.monotonic() + 5.0
        while not robot.enable():
            if time.monotonic() > t_end:
                raise RuntimeError("nero enable 타임아웃")
            time.sleep(0.01)
        robot.set_speed_percent(self.speed_percent)
        robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
        self.robot = robot
        return f"nero 연결 ({self.channel}, {robot.joint_nums}축, speed {self.speed_percent}%)"

    def close(self) -> None:
        if self.robot is not None:
            try:
                self.robot.disable()
            finally:
                self.robot = None

    def estop(self) -> None:
        if self.robot is not None:
            self.robot.electronic_emergency_stop()

    @property
    def connected(self) -> bool:
        return self.robot is not None

    def read_joint_angles(self) -> Optional[List[float]]:
        if self.robot is None:
            return None
        try:
            fb = self.robot.get_joint_angles()
            if fb is None:
                return None
            return [float(v) for v in getattr(fb, "msg", fb)]
        except Exception:
            return None

    def send(self, q: Dict[str, float]) -> None:
        if self.robot is None:
            return
        vals = [float(q[n]) for n in self.nero_joints if n in q]
        if len(vals) != len(self.nero_joints):
            return
        while len(vals) < self.robot.joint_nums:
            vals.append(0.0)
        self.robot.move_j(vals[:self.robot.joint_nums])


def attach_safety(model, robot, rate_hz: float):
    from whatslab.safety import SafetyFilter, load_limits_from_urdf, tighten

    v = robot.rig.solver.max_joint_velocity
    if not v:
        return None
    limits = {}
    for spec in (robot.rig.arm, robot.rig.hand):
        if spec is None:
            continue
        try:
            with open(spec.urdf_abspath(), encoding="utf-8") as f:
                limits.update(load_limits_from_urdf(f.read()))
        except Exception:
            pass
    limits = tighten(limits, {n: {"velocity": float(v)} for n in robot.arm_joint_names})
    model.safety = SafetyFilter(limits, dt=1.0 / rate_hz)
    return model.safety


class RobotBridge:
    def __init__(self, model, robot, args):
        self.model, self.robot, self.args = model, robot, args
        self.arm = self.hand = None
        self.sending = False
        self.status = None

    def connect_arm(self) -> str:
        s = AgxArmSender(self.robot.arm_joint_names, channel=self.args.can,
                         speed_percent=self.args.speed)
        msg = s.connect()
        self.arm = s
        fb = s.read_joint_angles()
        ik = self.model.ik.get(self.args.side)
        if fb is not None and ik is not None:
            q_now = list(ik._robot.solver.history_data)
            for i in range(min(len(s.nero_joints), len(fb), len(q_now))):
                q_now[i] = fb[i]
            ik.sync_state(q_now)
            msg += " / warm-start 동기화"
        return msg

    def connect_hand(self) -> str:
        carpal = next((n for n in self.robot.arm_joint_names
                       if not n.startswith("joint")), None)
        s = OrcaHandSender(side=self.args.side, wrist_joint=carpal)
        msg = s.connect()
        self.hand = s
        return msg

    def disconnect(self) -> None:
        self.sending = False
        for s in (self.arm, self.hand):
            if s is not None:
                s.close()
        self.arm = self.hand = None

    def estop(self) -> None:
        self.sending = False
        if self.arm is not None:
            self.arm.estop()
        if self.model.safety is not None:
            self.model.safety.trip()

    def send(self, q) -> None:
        if not self.sending or not q:
            return
        for s in (self.arm, self.hand):
            if s is None or not s.connected:
                continue
            try:
                s.send(q)
            except Exception as e:
                self.sending = False
                if self.status is not None:
                    self.status.content = f"**전송 오류 → 송신 중단**: `{e}`"


def build_robot_panel(model, robot, args) -> RobotBridge:
    from whatslab.viz import get_server

    bridge = RobotBridge(model, robot, args)
    srv = get_server(args.port)
    with srv.gui.add_folder("실물 로봇"):
        bridge.status = srv.gui.add_markdown("미연결")
        b_arm = srv.gui.add_button("팔 연결 (nero)")
        b_hand = srv.gui.add_button("손 연결 (orca)")
        cb_send = srv.gui.add_checkbox("송신", initial_value=False)
        b_stop = srv.gui.add_button("E-STOP", color="red")
        b_off = srv.gui.add_button("전체 해제")

    def _say(msg):
        bridge.status.content = msg
        print(f"[robot] {msg}", flush=True)

    @b_arm.on_click
    def _(_e):
        try:
            _say(bridge.connect_arm())
        except Exception as e:
            _say(f"**팔 연결 실패**: `{e}`")

    @b_hand.on_click
    def _(_e):
        try:
            _say(bridge.connect_hand())
        except Exception as e:
            _say(f"**손 연결 실패**: `{e}`")

    @cb_send.on_update
    def _(_e):
        if cb_send.value and bridge.arm is None and bridge.hand is None:
            cb_send.value = False
            _say("**먼저 연결하세요**")
            return
        bridge.sending = bool(cb_send.value)
        _say("송신 중" if bridge.sending else "송신 정지")

    @b_stop.on_click
    def _(_e):
        bridge.estop()
        cb_send.value = False
        _say("**E-STOP** — 해제 후 재연결")

    @b_off.on_click
    def _(_e):
        bridge.disconnect()
        cb_send.value = False
        _say("미연결")

    return bridge


class Diag:
    def __init__(self, robot, model, side, window=0.5):
        self.robot, self.model, self.side, self.window = robot, model, side, window
        self.t0 = None
        self.reset()

    def reset(self):
        self.n = self.n_in = self.n_clamp = 0
        self.in_p = self.tgt_p = self.base_p = 0.0
        self.pe = self.oe = 0.0
        self.dq_max = 0.0
        self.q_prev = None

    def tick(self, raw_pose, q_arm, now):
        import numpy as np
        from whatslab.robot.model import clamp_reach
        if self.t0 is None:
            self.t0 = now
        self.n += 1
        if raw_pose is not None:
            self.n_in += 1
            self.in_p = max(self.in_p, float(np.linalg.norm(np.asarray(raw_pose.pos))))
        T = self.model.target.get(self.side)
        rm = self.robot.rig.solver.reach_max
        if T is not None:
            self.tgt_p = max(self.tgt_p, float(np.linalg.norm(T[:3, 3])))
            T_b = self.robot.to_base(T)
            n_b = float(np.linalg.norm(T_b[:3, 3]))
            self.base_p = max(self.base_p, n_b)
            if rm and n_b > rm:
                self.n_clamp += 1
            if q_arm is not None:
                pe, oe = self.robot.solver.pose_error(q_arm, clamp_reach(T_b, rm))
                self.pe, self.oe = max(self.pe, pe), max(self.oe, oe)
        if q_arm is not None:
            if self.q_prev is not None:
                self.dq_max = max(self.dq_max, float(np.linalg.norm(q_arm - self.q_prev)))
            self.q_prev = np.array(q_arm, dtype=float)
        if now - self.t0 < self.window:
            return
        c = self.model.calib.get(self.side)
        flags = "W" if (c is not None and c.ready) else "-"
        print(f"[diag] in {self.n_in:3d}/{self.n:3d} |p|{self.in_p:5.2f}m  "
              f"tgt|p|{self.tgt_p:5.2f}  base|p|{self.base_p:5.2f} clamp "
              f"{self.n_clamp*100//max(1,self.n):3d}%  err {self.pe*1000:6.1f}mm/"
              f"{np.degrees(self.oe):5.1f}deg  dq {self.dq_max:5.3f}  calib {flags}",
              flush=True)
        self.t0 = now
        self.reset()
