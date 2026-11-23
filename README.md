<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%" ></p>

<h1 align="center">whatslab</h1>

<p align="center"><b>English</b> | <a href="README.ko.md">한국어</a></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python"/>
  <img src="https://img.shields.io/badge/Spine-3.0.0-green.svg" alt="Spine"/>
</p>

**whatslab** is the teleoperation core of WHATs LAB Corp. It is a pure-Python SDK that
turns human motion (VR controllers, hand tracking, data gloves) into robot arm and hand
joint angles, and it is the shared logic layer beneath our simulators (MuJoCo, Isaac Sim)
and ROS 2 stack.

whatslab does not depend on ROS and runs in-process anywhere. It provides the *parts*
(input receivers, calibration, hand/arm retargeting, visualization, dataset recording) and
leaves the *assembly* (wiring them into a simulator or robot pipeline) to you. Every input
is normalized to one canonical frame (x forward, z up, right-handed), so downstream code
never re-maps axes.

## Table of contents

- [Main features](#main-features)
- [Installation](#installation)
- [Compatibility](#compatibility)
- [Quick start](#quick-start)
- [Examples & tools](#examples--tools)
- [Documentation](#documentation)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## Main features

**Framework-agnostic**
- Pure Python with no ROS dependency; used in-process from MuJoCo, Isaac Sim or ROS 2
- Provides composable parts; you own the pipeline
- Strict dependency direction (`receiver → core`, `robot → core·solvers`, `teleop → core·robot·solvers`)

**Retargeting-first**
- Hand: one input, the OpenXR 26-joint hand (`HandPose.xr_joints`), from the AirGlove or
  from Quest hand tracking, and two engines:
  - DexPilot vector retargeting ([dex-retargeting](https://github.com/dexsuite/dex-retargeting)),
    1–2 ms per hand per frame, any robot hand with a URDF
  - the **GeoRT soft-prompt** network: one forward pass in numpy, about 0.2 ms, one trained
    model shared by several robot hands
- Arm: pinocchio analytic Jacobian + damped least squares
- Output is `{side: {joint_name: rad}}`, ready to publish

**Calibrated**
- Wrist yaw alignment (head-relative snapshot)
- Per-user arm reach scaling, saved into the rig config for the next session

### Supported hands

A robot config's `retarget:` picks the engine and the hand.

**DexPilot** — `retarget: dex:<name>`, where `<name>` is a file in `configs/hands/`
(`dex:orca` resolves to `orca_{side}`; a file path also works). A config for one side
serves the other side by mirroring the input.

| Hand | `retarget:` | Config |
|---|---|---|
| ORCA Hand | `dex:orca` | `orca_{left,right}.yaml` (wrist joint excluded, held at 0) |
| Tesollo DG-5F | `dex:dg5f` | `dg5f_{left,right}.yaml` (PIP/DIP limited to −5..90°) |
| ROBOTIS HX5-D20 | `dex:robotis` | `robotis_{left,right}.yaml` |
| Allegro Hand | `dex:allegro` | `allegro_{left,right}.yaml` (4 fingers; the human little finger is not used) |
| LEAP Hand | `dex:leap` | `leap_{left,right}.yaml` (4 fingers; the human little finger is not used) |
| PSYONIC Ability Hand | `dex:ability` | `ability_{left,right}.yaml` (6 actuated joints; PIP follows the URDF mimic) |
| Schunk SVH | `dex:schunk` | `schunk_{left,right}.yaml` (9 actuated joints; coupled joints follow the URDF mimic) |

**GeoRT soft prompt** — `retarget: geort:<name>`, where `<name>` is a file in
`configs/geort/` (a path ending in `.yaml` also works). The trained model ships with the
package (`solvers/hand/geort/assets/geort_prompt.npz`, about 5 MB); a config's `model:` or
`WHATSLAB_GEORT_MODEL` overrides it. The model is right-handed; a left hand is mirrored onto
it. On a rig, the output is also clipped to the robot hand's URDF joint limits.

| Hand | `retarget:` | URDF in `dexhand-description` |
|---|---|---|
| ORCA Hand (right) | `geort:orca_right` | `orca_hand/urdf/right.urdf` |
| ROBOTIS HX5-D20 (right) | `geort:hx5_right` | `robotis_hx5_d20/urdf/hx5_d20_right.urdf` |

The model also holds Allegro and DG-5F-S prompts, trained against URDFs that are not in
dexhand-description yet. Training tools are not part of this SDK.

## Installation

Published under a source-available license. It is not on PyPI; install from source.

```bash
pip install '.[all]'      # receiver + hand + arm + viz + data
pip install '.[hand]'     # or any subset: receiver / hand / arm / viz / data
pip install -e '.[all]'   # editable, for development
```

| Extra | Brings in | Needed for |
|---|---|---|
| `receiver` | `airglove-client` (git tag v0.4.0) | AirGlove (Spine) input. WebXR input needs nothing extra. |
| `hand` | `pin`, `dex-retargeting>=0.4.6`, `dexhand-description` | the DexPilot engine |
| `arm` | `pin`, `dexhand-description` | arm IK, rigs, `whatslab.teleop` presets |
| `viz` | `viser`, `trimesh`, `pin` | robot visualization, `tools/align_frames.py` |
| `data` | `pyarrow`, `imageio[ffmpeg]` | LeRobot v2.1 dataset recording |

numpy has no upper bound. dex-retargeting 0.4.6 is the last release for numpy 1 and 0.5.0
is the first for numpy 2, so pip picks the one that matches your environment; numpy-1
consumers (Isaac Sim 5.1) pin `numpy<2` and `dex-retargeting<0.5` on their side.
dex-retargeting imports `torch` at module level and needs `nlopt<2.8`, `pin>=2.7` and
Python < 3.13. GeoRT inference itself (`whatslab.solvers.hand.geort`) needs only numpy.

Robot and rig configs are bundled. URDFs and meshes come from the separate package
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf) (pulled in by the
`hand`/`arm` extras); point `WHATSLAB_MODELS_ROOT` at another asset tree to override it.

Teleoperating real hardware (Nero arm + ORCA hand) needs drivers that are not package
dependencies. `scripts/install_robot_deps.sh` installs them in one step:

```bash
PY=$(which python) ./scripts/install_robot_deps.sh
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --viz --robot
```

`pyAgxArm` (Nero arm, CAN) is listed in `examples/requirements-robot.txt`; `orca_core` (ORCA hand)
is cloned and installed editable by the script, for reasons given in the
[quickstart](docs/QUICKSTART.md#1-설치).

## Compatibility

| Input | Needs |
|---|---|
| AirGlove | **Spine 3.0.0** (OSC protocol 3) running on the same machine |
| Quest controllers / hand tracking | the headset's browser (WebXR); no app to install, no Spine |

**Spine and the AirGlove.** `AirGloveReceiver`, the default glove source of `GloveModel`
and `HandModel`, wraps [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client)
(`airglove-client` 0.4.0), which speaks Spine 3.0.0's OSC protocol 3. Spine 2.x is not supported
since whatslab 0.5.0; use whatslab 0.4.x for it. Spine sends to one client per machine (UDP 4040),
so only one process on the Spine PC can receive the glove; `AirGloveReceiver(pair=1)` selects the
Spine pair. Per hand it delivers `HandPose.xr_joints`, the 26 OpenXR joints as a `(26, 8)` array
relative to the wrist. `HandPose.wrist` is always `None` (Spine sends only an uncalibrated IMU
quaternion), so `GloveModel` takes the arm rotation from the controller. Field-level details,
freshness and the extra client calls are in [API.md](docs/API.md) (OpenXR glove).

**Hand retargeting** reads `xr_joints` only: `dex:<name>` maps them to the 21-point human hand
aligned to the robot's palm frame, `geort:<name>` feeds the same 21 points to the GeoRT network.

## Quick start

```python
from whatslab.teleop import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # arm = WebXR controller → IK, hand = AirGlove → retargeting
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — arm + hand merged
    publish_joint_states(q)   # your job: reorder into your sim/ROS joint order
```

Presets: `QuestModel` (Quest bare-hand tracking), `GloveModel` (controller + glove) and
`HandModel` (hand only). Receiver options go in as dicts:
`GloveModel(rig, hand_kwargs={"glove_port": 4040}, arm_kwargs={"port": 8443})`,
`HandModel(hand_kwargs={...})`.

Quest input comes only through the headset's browser (WebXR); there is no app to sideload.
Controllers (`WebXRControllerReceiver`) and bare-hand tracking (`WebXRHandReceiver`) share one
port, over self-signed HTTPS by default (`tls=False` for wired `adb reverse`). See
[API.md](docs/API.md) (WebXR path).

For your own hardware combination, subclass `TeleopModel` and implement the one abstract hook,
`_get_raw_target()`, which decides which source feeds the arm end-effector target. Calibration,
IK, retargeting and the safety filter are already wired.

## Examples & tools

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml              # controller + glove
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest bare-hand tracking
python examples/airglove_hand.py --config geort:orca_right                # glove → robot hand only
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml             # inspect rig kinematics

python tools/align_frames.py robot --robot robots/nero.yaml               # align a robot to the canonical axes
```

Run the tests with `pip install -e '.[all,dev]' && pytest`.

## Documentation

- [**Real-robot quickstart**](docs/QUICKSTART.md) (Korean) — from clone to teleoperating a Nero
  arm + ORCA hand, and what usually goes wrong
- [**Guide**](docs/GUIDE.md) (Korean) — bringing up a new robot, calibration, arm-IK tuning and how
  to judge a change, diagnostics, sending to real hardware
- [**API reference**](docs/API.md) (Korean) — public symbols per subpackage, with signatures
- [**Changelog**](CHANGELOG.md) (Korean) — version history. **0.5.0 contains breaking changes**
  (Spine 3.0.0 only, `AirGloveReceiver`, `dex:`/`geort:` retargeting, Quest via WebXR only).

## Acknowledgments

whatslab builds on excellent open-source work:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (rigid-body kinematics/IK),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting) (DexPilot hand retargeting, MIT),
[GeoRT](https://zhaohengyin.github.io/geort) (geometric hand retargeting, which the soft-prompt
model's training builds on),
[viser](https://github.com/nerfstudio-project/viser) (web 3D visualization) and
[LeRobot](https://github.com/huggingface/lerobot) (dataset format).

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0): source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
