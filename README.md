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
- hand: human-hand URDF joint angles from the glove → pinocchio FK → retargeting IK.
  `backend="dex"` (dex-retargeting two-stage vector+position) or `backend="kp"`
  (combined keypoint objective: palm-relative fingertips + shape + DexPilot-style
  pinch snap, pin+numpy only)
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

## Compatibility

Data-glove teleoperation goes through **Spine**, WHATs LAB's glove middleware.
Supported Spine versions: **2.3.1 and below** (newer versions are not yet supported).
Controller / hand-tracking (Quest) paths do not require Spine.

The glove path follows Spine's OSC contract as documented in Spine's
`docs/OSC_Protocol.md`: `GloveHumanAnglesReceiver` consumes
`/{side}/joint_angles/get` (name/angle pairs) plus `/{side}/wrist/get`, and
`GloveRobotHandReceiver` consumes the same addresses for the IK-bypass path. Every
Spine message carries the message type in `args[0]`. `wrist` is passed through in
Spine's own frame with no conversion.

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
python tools/bench_arm_ik.py --traj fk                                 # arm IK: accuracy / continuity / cost
python tools/bench_hand_retarget.py --dump … --profiles … --traj flex  # hand retargeting: shape / GMC / LMC
python tools/train_hand_net.py --config orca_hand --side left …        # train the `net` backend
python tools/check_mirror.py                                           # left/right mirror gate
python tools/export_hand_net_onnx.py --checkpoint … --out hand.onnx    # ONNX: no torch/pinocchio at inference
```

`bench_hand_retarget.py` is the fixed baseline for hand-retargeting changes. Judge on
`--traj flex` (flexion-only sweep) as well as `pinch` — a pinch ramp is monotonic, so it
hides the lateral wander that shows up during pure flexion. `LMC` (per-finger local
frame) is the primary number and `GMC` (shared frame) is reported alongside; the gap
between them diagnoses alignment quality. Clear `__pycache__` before measuring.

#### `net` backend baseline

The baseline for the hand-retargeting `net` backend is **`c2c7967`** — the first commit
that reads `prox` from the proximal sensor frame, and the code the models below were
trained with. Losses `motion 1 / coverage 5 / bone 20 / pinch 1 / pos 20` (code
constants), 1000 epochs, fp32, `--random-mode mix` plus recorded human `q`. Left hand,
against `kp`:

| | traj | shape° | tip° | open-contact | GMC | LMC | \|dq\|p95 | ms |
|---|---|---|---|---|---|---|---|---|
| robotis kp | flex | 33.9 | 38.9 | **4.6** | **95.4** | 78.4 | **0.115** | 1.93 |
| robotis net | flex | **27.5** | **23.6** | 11.7 | 88.5 | **85.7** | 0.380 | **0.28** |
| robotis kp | abd | **7.5** | 8.3 | **7.2** | 99.7 | 99.2 | **0.032** | 1.93 |
| robotis net | abd | 11.0 | 8.6 | 11.0 | 99.7 | 99.1 | 0.037 | **0.27** |
| orca kp | flex | **46.7** | **41.8** | **1.6** | **96.6** | 77.1 | **0.111** | 2.20 |
| orca net | flex | 55.3 | 48.9 | 16.2 | 83.7 | **82.2** | 0.181 | **0.27** |
| orca kp | abd | **11.0** | 11.8 | **11.9** | 88.2 | 88.3 | **0.036** | 2.19 |
| orca net | abd | 12.9 | **11.7** | 11.1 | **91.6** | **92.2** | 0.150 | **0.27** |

`net` wins on **LMC in all four cases**, on robotis flexion shape/tip (27.5/23.6 vs
33.9/38.9), and on inference cost (7-8x). `net` loses on **open-hand contact (11-16mm
vs `kp` 1.6-7.2** — the hand does not fully extend), on **`|dq|p95` (up to 3x** — it
judders), and on orca flexion shape/tip. Compare later changes against this table.

**The LMC definition changed.** The old one pinned the local frame's roll to the palm y
axis, so a robot whose fingers splay at neutral was penalized for correct motion (orca's
abduction 41.0 was a fixed 50-degree roll offset). It now aligns the bone axes by minimal
rotation before comparing directions, which is frame-independent. LMC recorded before
`c2c7967` is not comparable to this table.

Left/right handling is per-hand — the deciding measurement is that hand's **left/right
URDF mirror fidelity**. orca's two URDFs are exact mirrors (0.86mm, limits identical on
all 16 joints), so one left-trained model covers both sides via `hand_solver.mirror_to`
(right-hand fingertip tracking 24.49 vs 24.02mm for a dedicated model). robotis has
mismatched conventions on `finger_joint1~4`, where mirroring is 63% worse (49.65 vs
30.50mm), so **per-side checkpoints are required**. The training side is stamped into
the checkpoint and checked on load, so swapping left/right raises an error.

`bench_arm_ik.py` is the fixed baseline for arm-IK changes. Judge accuracy on `--traj fk`
(targets generated by FK from valid joint angles, so the error floor is exactly 0);
coordinate-space trajectories pass through unreachable poses and conflate solver quality
with reachability. `--floor` reports the achievable lower bound for the worst frames.

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
[dex-retargeting](https://github.com/dexsuite/dex-retargeting) (hand retargeting),
[viser](https://github.com/nerfstudio-project/viser) (web 3D visualization), and
[LeRobot](https://github.com/huggingface/lerobot) (dataset format).

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0) — source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
