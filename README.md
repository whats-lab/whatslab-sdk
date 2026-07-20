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

**whatslab is pip-only:**
- hand, arm, receiver and viz all share one pip `pinocchio` (double `pin`) — no conda-forge
- arm IK uses an analytic Jacobian + DLS, not casadi/IPOPT
- pinned to `numpy<2` for consistency across the stack (dex-retargeting, Isaac Sim 5.1)

**whatslab is retargeting-first:**
- hand: dex-retargeting two-stage (vector + position) IK with deterministic termination
- arm: pinocchio analytic Jacobian + damped least squares
- output is `{side: {joint_name: rad}}`, ready to publish

**whatslab is calibrated:**
- wrist yaw alignment (head-relative snapshot)
- per-user arm reach scaling, persisted into the rig config

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

## Quick start

```python
from whatslab.model import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # arm = controller IK, hand = glove retarget
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — arm + hand merged
    publish_joint_states(q)   # consumer's job: reorder into sim/ROS joint order
```

Presets: `QuestModel` (hand-tracking), `GloveModel` (controller + glove),
`HandModel` (hand only). For custom hardware combinations, subclass `TeleopModel`
and override `get_data()`.

## Examples & tools

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml            # controller + glove
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest hand-tracking
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml           # inspect rig kinematics

python tools/align_frames.py robot --robot robots/nero.yaml            # align a robot to canonical axes
```

Run the test suite with `pip install -e '.[all,dev]' && pytest`.

## Documentation

- [**API reference**](docs/API.md) — public symbols per subpackage, with signatures.

## Acknowledgments

whatslab builds on excellent open-source work:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (rigid-body kinematics/IK),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting) (hand retargeting),
[viser](https://github.com/nerfstudio-project/viser) (web 3D visualization), and
[LeRobot](https://github.com/huggingface/lerobot) (dataset format).

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0) — source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
