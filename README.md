<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%" ></p>

<h1 align="center">whatslab</h1>

<p align="center"><b>English</b> | <a href="README.ko.md">한국어</a></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python"/>
  <img src="https://img.shields.io/badge/numpy-%3C2-blue.svg" alt="numpy"/>
  <img src="https://img.shields.io/badge/tests-88%20passing-brightgreen.svg" alt="Tests"/>
</p>

**whatslab** is the WHATs LAB teleoperation core — a pure-Python SDK that turns human
motion (VR controllers, hand-tracking, data gloves) into robot arm and hand joint
angles. It is developed at WHATs LAB as the shared logic layer beneath our simulators
(MuJoCo, Isaac Sim) and ROS2 stack.

whatslab has no dependency on ROS and runs in-process anywhere. It provides the *parts* —
input receivers, calibration, hand/arm retargeting, visualization, dataset recording —
and leaves the *assembly* (wiring a pipeline into a simulator or robot) to the consumer.
Inputs are normalized to a single canonical frame (x=forward, z=up, right-handed), so
downstream code never re-maps axes.

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

**whatslab is framework-agnostic:**
- pure Python, zero ROS dependency — used in-process from MuJoCo, Isaac Sim, or ROS2
- provides composable parts; the consumer owns the pipeline
- strict dependency direction (`receiver → core`, `model → core·robot`)

**whatslab is retargeting-first:**
- hand: human-hand URDF joint angles from the glove → one forward pass of a single
  learned ONNX model. No per-frame IK — 900Hz+ on one CPU thread, every robot hand
  and both sides share one graph
- arm: pinocchio analytic Jacobian + damped least squares
- output is `{side: {joint_name: rad}}`, ready to publish

**whatslab is calibrated:**
- wrist yaw alignment (head-relative snapshot)
- per-user arm reach scaling, persisted into the rig config

### Supported hands

One ONNX graph covers every hand below, both sides. The name in the first column is
what a rig's `retarget:` field takes; the alias in parentheses also resolves.

| Hand | `retarget:` | URDF in `dexhand-description` |
|---|---|---|
| Human reference hand (retargeting input) | `human` (`base_hand`) | `base_hand/urdf/{side}.urdf` |
| ORCA Hand | `orca` (`orca_hand`) | `orca_hand/urdf/{side}.urdf` |
| Allegro Hand | `allegro` (`allegro_hand`) | `allegro_hand/allegro_hand_{side}.urdf` |
| Tesollo DG-5F | `tesollo` (`tesollo_dg5f`) | `tesollo_dg5f/dg5f_{side}.urdf` |
| ROBOTIS HX5-D20 | `robotis` (`robotis_hx5_d20`) | `robotis_hx5_d20/urdf/hx5_d20_{side}.urdf` |

Any other hand raises on construction, listing the names the table does hold. To add
one, generate its tables with retarget_net's `tools/onboard_urdf.py` — the URDF must
carry the sensor-frame contract (`{side}_sensor_dorsum` plus `_proximal`/`_distal` per
finger). A hand the model was not trained on needs a few hundred fine-tuning steps and
a graph re-export before it performs.

## Installation

Publicly available under a source-available license (not published on PyPI — install from source).

```bash
pip install '.[all]'      # receiver + hand + arm + viz
pip install '.[hand]'     # partial: hand / arm / receiver / viz / data
pip install -e '.[all]'   # editable, for development
```

Robot/rig configs are bundled. URDF and meshes are provided by the separate
single-source package [`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf),
pulled in by the `hand`/`arm` extras; override the asset tree with `WHATSLAB_MODELS_ROOT`.

## Compatibility

Data-glove teleoperation goes through **Spine**, WHATs LAB's glove middleware.
Supported Spine versions: **2.3.1 and below** (newer versions are not yet supported).
Controller / hand-tracking (Quest) paths do not require Spine.

The glove path follows Spine's OSC contract as documented in Spine's
`docs/OSC_Protocol.md`: `GloveHumanHandReceiver` consumes `/{side}/quat/get`, and
`GloveRobotHandReceiver` consumes `/{side}/joint_angles/get` (name/angle pairs) plus
`/{side}/wrist/get`. Every Spine message carries the message type in `args[0]`.

## Quick start

```python
from whatslab.teleop import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # arm = controller IK, hand = glove retarget
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — arm + hand merged
    publish_joint_states(q)   # consumer's job: reorder into sim/ROS joint order
```

Presets: `QuestModel` (hand-tracking), `GloveModel` (controller + glove),
`HandModel` (hand only). For a custom hardware combination, subclass `TeleopModel`
and implement the single abstract hook `_get_raw_target()` — it decides which source
feeds the arm EE target. Everything else (calibration, IK, retargeting, safety) is
already wired.

## Examples & tools

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml            # controller + glove
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest hand-tracking
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml           # inspect rig kinematics

python tools/align_frames.py robot --robot robots/nero.yaml            # align a robot to canonical axes
```

Judge arm-IK accuracy on targets generated by FK from valid joint angles — the error
floor is then exactly 0, so whatever error remains is the solver's. Coordinate-space
trajectories pass through unreachable poses and conflate solver quality with
reachability. (The fixed arm-IK benchmark is internal tooling and is not shipped here.)

Run the test suite with `pip install -e '.[all,dev]' && pytest`.

## Documentation

- [**Guide**](docs/GUIDE.md) — bringing up a new robot, calibration workflow, arm-IK
  tuning and how to judge a change, diagnostics, sending to real hardware.
- [**API reference**](docs/API.md) — public symbols per subpackage, with signatures.
- [**Changelog**](CHANGELOG.md) — version history. **0.2.0 contains breaking changes**
  (`model.ik[s]` → `model.sides[s].ik`, `RobotModel.solve` removed).

## Acknowledgments

whatslab builds on excellent open-source work:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (rigid-body kinematics/IK),
[viser](https://github.com/nerfstudio-project/viser) (web 3D visualization), and
[LeRobot](https://github.com/huggingface/lerobot) (dataset format).

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0) — source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
