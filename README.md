<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%" ></p>

<h1 align="center">whatslab</h1>

<p align="center"><b>English</b> | <a href="README.ko.md">한국어</a></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python"/>
  <img src="https://img.shields.io/badge/Spine-3.0.0-green.svg" alt="Spine"/>
</p>

**whatslab** is the hand SDK of WHATs LAB Corp. It is a pure-Python library that turns a
human hand (the AirGlove data glove, or Quest hand tracking) into robot-hand joint angles.

whatslab does not depend on ROS and runs in-process anywhere (MuJoCo, Isaac Sim, ROS 2).
Its scope is the hand: input receivers, hand retargeting, a joint safety filter and
visualization. **Arm IK, wrist tracking and arm control are not part
of it** — the SDK gives hand joints relative to the wrist; get the wrist pose from your own tracker
(Vive, Quest controller, motion capture) and pass it to your arm controller.

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

- One hand input for every source: the OpenXR 26-joint hand (`HandPose.xr_joints`), from the
  AirGlove (through Spine) or from Quest hand tracking (headset browser, WebXR)
- Two retargeting engines:
  - DexPilot vector retargeting ([dex-retargeting](https://github.com/dexsuite/dex-retargeting)),
    1–2 ms per hand per frame, any robot hand with a URDF
  - the **GeoRT soft-prompt** network: one forward pass in numpy, about 0.2 ms, one trained
    model shared by several robot hands
- Output is `{side: {joint_name: rad}}`, ready to publish; an optional `SafetyFilter` clamps
  position and rate
- Pure Python, no ROS dependency; strict dependency direction
  (`receiver → core`, `solvers → core`, `teleop → core·receiver·solvers`)

### Supported hands

**DexPilot** — `dex:<name>`, where `<name>` is a file in `configs/hands/` (`dex:orca` resolves
to `orca_{side}`; a file path also works). A config for one side serves the other side by
mirroring the input.

| Hand | Engine name | Config |
|---|---|---|
| ORCA Hand | `dex:orca` | `orca_{left,right}.yaml` (wrist joint excluded, held at 0) |
| Tesollo DG-5F | `dex:dg5f` | `dg5f_{left,right}.yaml` (PIP/DIP limited to −5..90°) |
| ROBOTIS HX5-D20 | `dex:robotis` | `robotis_{left,right}.yaml` |
| Allegro Hand | `dex:allegro` | `allegro_{left,right}.yaml` (4 fingers; the human little finger is not used) |
| LEAP Hand | `dex:leap` | `leap_{left,right}.yaml` (4 fingers; the human little finger is not used) |
| PSYONIC Ability Hand | `dex:ability` | `ability_{left,right}.yaml` (6 actuated joints; PIP follows the URDF mimic) |
| Schunk SVH | `dex:schunk` | `schunk_{left,right}.yaml` (9 actuated joints; coupled joints follow the URDF mimic) |

**GeoRT soft prompt** — `geort:<name>`, where `<name>` is a file in `configs/geort/` (a path
ending in `.yaml` also works). The trained model ships with the package
(`solvers/hand/geort/assets/geort_prompt.npz`, about 5 MB); a config's `model:` or
`WHATSLAB_GEORT_MODEL` overrides it. The model is right-handed; a left hand is mirrored onto it.

| Hand | Engine name | URDF in `dexhand-description` |
|---|---|---|
| ORCA Hand (right) | `geort:orca_right` | `orca_hand/urdf/right.urdf` |
| ROBOTIS HX5-D20 (right) | `geort:hx5_right` | `robotis_hx5_d20/urdf/hx5_d20_right.urdf` |

The model also holds Allegro and DG-5F-S prompts, trained against URDFs that are not in
dexhand-description yet. Training tools are not part of this SDK.

## Installation

Published under a source-available license. It is not on PyPI; install from source.

```bash
pip install '.[all]'      # receiver + hand + viz + webxr
pip install '.[receiver]' # or any subset: receiver / hand / viz / webxr
pip install -e '.[all]'   # editable, for development
```

| Extra | Brings in | Needed for |
|---|---|---|
| `receiver` | `airglove-client` (git tag v0.4.0) | AirGlove (Spine) input |
| `webxr` | `websockets` | Quest hand tracking (headset browser) |
| `hand` | `pin`, `dex-retargeting>=0.4.6`, `dexhand-description` | the DexPilot engine |
| `viz` | `viser`, `trimesh`, `pin` | robot-hand visualization |

GeoRT inference (`geort:`) needs only numpy. dex-retargeting 0.4.6 is the last release for
numpy 1 and 0.5.0 is the first for numpy 2, so pip picks the one that matches your
environment; it imports `torch` at module level and needs `nlopt<2.8`, `pin>=2.7` and
Python < 3.13. URDFs and meshes come from
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf) (pulled in by the
`hand` extra); point `WHATSLAB_MODELS_ROOT` at another asset tree to override it.

## Compatibility

| Input | Needs |
|---|---|
| AirGlove | **Spine 3.0.0** (OSC protocol 3) running on the same machine |
| Quest hand tracking | the headset's browser (WebXR); no app to install, no Spine |

`AirGloveReceiver` wraps [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client)
(`airglove-client` 0.4.0). Spine 2.x is not supported since whatslab 0.5.0; use whatslab 0.4.x
for it. Spine sends to one client per machine (UDP 4040), so only one process on the Spine PC
can receive the glove. Per hand it delivers `HandPose.xr_joints`, the 26 OpenXR joints as a
`(26, 8)` array relative to the wrist. No wrist pose is given: what Spine
sends for the wrist is an uncalibrated IMU quaternion, not a wrist pose. Field-level details are in
[API.md](docs/API.md) (OpenXR glove).

Only need the raw glove data, another language (C/C++, C#, Unity, Unreal) or your own
retargeting? Use [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client) directly. For
ROS 2, see [whatslab-ros2](https://github.com/whats-lab/whatslab-ros2).

Recording is left to your pipeline: the SDK gives plain data (OpenXR 26-joint hands,
`{joint_name: rad}`, timestamps), so it fits LeRobot, HDF5 or your own format; on ROS 2,
record the whatslab-ros2 topics with rosbag2.

## Quick start

```python
from whatslab.teleop import HandModel

m = HandModel("geort:orca_right", side="right")   # AirGlove → ORCA right hand
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}; {} while the hand is not tracked
    publish_joint_states(q)   # your job: reorder into your sim/ROS joint order
```

Quest hand tracking instead of the glove:

```python
from whatslab.receiver import WebXRHandReceiver
m = HandModel("dex:orca", hand_source=WebXRHandReceiver())   # open the printed https URL in the headset
```

## Examples & tools

```bash
python examples/airglove_hand.py --config geort:orca_right               # glove → robot hand, prints q
python examples/airglove_hand.py --config dex:orca --viz                 # with the robot hand in viser
python tools/bench_hand_retarget.py --help                               # retargeting quality metrics
```

Run the tests with `pip install -e '.[all,dev]' && pytest`.

## Documentation

- [**Quickstart**](docs/QUICKSTART.md) (Korean) — from clone to a robot hand following the glove,
  and what usually goes wrong
- [**API reference**](docs/API.md) (Korean) — public symbols per subpackage, with signatures

## Acknowledgments

whatslab builds on excellent open-source work:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (kinematics),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting) (DexPilot hand retargeting, MIT),
[GeoRT](https://zhaohengyin.github.io/geort) (geometric hand retargeting, which the soft-prompt
model's training builds on),
and [viser](https://github.com/nerfstudio-project/viser) (web 3D visualization).

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0): source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
