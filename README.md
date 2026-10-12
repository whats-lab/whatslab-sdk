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
- hand: the glove's OpenXR 26-joint skeleton → DexPilot vector retargeting
  ([dex-retargeting](https://github.com/dexsuite/dex-retargeting) 0.4.6), 1–2 ms
  per hand per frame. Legacy human-hand URDF joint angles still go through one
  forward pass of a learned ONNX model
- arm: pinocchio analytic Jacobian + damped least squares
- output is `{side: {joint_name: rad}}`, ready to publish

**whatslab is calibrated:**
- wrist yaw alignment (head-relative snapshot)
- per-user arm reach scaling, persisted into the rig config

### Supported hands

**Current glove (OpenXR `xr_joints`)** — a rig's `retarget:` takes `dex:<name>`;
`<name>` is a file in `configs/hands/` (`dex:orca` resolves to `orca_{side}`, a file
path also works). The opposite side of a one-sided config is served by mirroring the
input.

| Hand | `retarget:` | config |
|---|---|---|
| ORCA Hand | `dex:orca` | `orca_{left,right}.yaml` (wrist joint excluded, held at 0) |
| Tesollo DG-5F | `dex:dg5f` | `dg5f_{left,right}.yaml` (PIP/DIP clamped to −5..90°) |
| ROBOTIS HX5-D20 | `dex:robotis` | `robotis_{left,right}.yaml` |
| Allegro Hand | `dex:allegro` | `allegro_{left,right}.yaml` (4 fingers — the human pinky is not used) |
| LEAP Hand | `dex:leap` | `leap_{left,right}.yaml` (4 fingers — the human pinky is not used) |
| PSYONIC Ability Hand | `dex:ability` | `ability_{left,right}.yaml` (6 actuated joints; PIP follows URDF mimic) |
| Schunk SVH | `dex:schunk` | `schunk_{left,right}.yaml` (9 actuated joints; coupled joints follow URDF mimic) |

**Legacy joint-angle input (Spine ≤ 2.3.1)** — one ONNX graph covers every hand
below, both sides. The name in the first column is what `retarget:` takes; the alias
in parentheses also resolves.

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
pip install '.[all]'      # receiver + hand + arm + viz + data
pip install '.[hand]'     # partial: hand / arm / receiver / viz / data
pip install -e '.[all]'   # editable, for development
```

Teleoperating real hardware (nero arm + ORCA hand) needs extra drivers.
`scripts/install_robot_deps.sh` handles them in one step.

```bash
PY=$(which python) ./scripts/install_robot_deps.sh
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --viz --robot
```

`pyAgxArm` (CAN) ships in the `robot` extra, but `orca_core` cannot: every tag
declares `numpy>=2.2.6` while this repo pins `numpy<2`, so resolution fails (the
code itself runs fine on numpy 1.26). Its wheel also omits the hand configs
(`models/<version>/<model>/config.yaml`), so the script clones the source and
installs it editable. The hand model defaults to whatever `orca_core` picks;
pass `--hand-model` for a different one.

The `hand` extra pins `dex-retargeting==0.4.6` — the last release that accepts
`numpy<2`. It imports `torch` at module level and needs `nlopt<2.8`, `pin>=2.7` and
Python < 3.13, and `import whatslab.teleop` now takes about 0.9 s (0.25 s without it).

Robot/rig configs are bundled. URDF and meshes are provided by the separate
single-source package [`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf),
pulled in by the `hand`/`arm` extras; override the asset tree with `WHATSLAB_MODELS_ROOT`.

## Compatibility

Data-glove teleoperation goes through **Spine**, WHATs LAB's glove middleware.
Controller / hand-tracking (Quest) paths do not require Spine.

**Current Spine (OpenXR hand output)** — `GloveOpenXRReceiver`, the default glove
source of `GloveModel` and `HandModel`, wraps the
[`airglove-client`](https://github.com/whats-lab/AirGlove-Client) library (pulled in by
the `receiver` extra; client API in its `python/README.md`, wire contract in
AirGlove-Client-Core `docs/spine_protocol.md`). The current Spine speaks OSC protocol 3
(message-number header, `/pair/N` addresses), which needs `airglove_client` 0.4.0; the pinned
`airglove-client` v0.1.0 speaks protocol 1, so until v0.4.0 is tagged point
`AIRGLOVE_CLIENT_LIBRARY` at a 0.4.0 native library (same ABI as the pinned Python package).
`GloveOpenXRReceiver(pair=1)` selects the Spine pair. Per hand it delivers:

- `HandPose.xr_joints` — `(26, 8)` array, 26 OpenXR `XrHandJointEXT` joints
  (palm, wrist, thumb ×4, index..little ×5) as `[qx qy qz qw px py pz radius]`, as
  received: wrist space (wrist = identity), OpenXR joint axes (−Z along the bone,
  +Y back of hand, same rule for both hands), metres.
- `HandPose.wrist` — always `None`. Protocol 3 no longer sends the SDK's world wrist
  orientation, only the raw back-of-hand IMU quaternion (Spine reorders it to
  `[y, x, z, -w]`) in an uncalibrated sensor frame. Using it as an arm target would move the
  arm wrongly, so the receiver does not; `GloveModel` takes the arm rotation from the
  controller instead (one warning per side). The raw value is available, unconverted,
  through `GloveOpenXRReceiver.raw_wrist(side)`.

Freshness (`stale_timeout`, default 0.2 s) is the age since the client *received* the
frame; protocol 3 carries no Spine timestamp or sequence (`seq` / `sender_time_us` are
client-side counters and receive times).

Fingers are retargeted from `xr_joints` when the rig's `retarget:` is `dex:<name>`
(`XRHandRetargeter`, DexPilot). Positions are taken relative to the wrist, mapped to
MANO's 21 keypoints, and rotated so the human palm frame (wrist + index/middle/pinky
MCP) lands on the robot's q=0 palm frame every frame — wrist orientation never leaks
into the fingers. A rig still set to an ONNX name (`orca_hand`, ...) reads
`joint_angles`, which the current Spine does not send, so its hand holds the last
command.

**Legacy (Spine 2.3.1 and below)** — `GloveHumanAnglesReceiver`
(`/{side}/joint_angles/get` + `/{side}/wrist/get`), `GloveHumanHandReceiver`
(`/{side}/quat/get`) and `GloveRobotHandReceiver` follow Spine's old
`docs/OSC_Protocol.md` and receive nothing from the current Spine. Select them with
`GloveModel(rig, hand_source="angles")` / `"robot"`.

## Quick start

```python
from whatslab.teleop import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # arm = controller IK; with the default openxr
                                              # glove source the robot fingers do NOT move yet
                                              # (no xr_joints retargeter yet, see Compatibility)
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — arm + hand merged
    publish_joint_states(q)   # consumer's job: reorder into sim/ROS joint order
```

Presets: `QuestModel` (hand-tracking), `GloveModel` (controller + glove),
`HandModel` (hand only). Receiver options go in as dicts —
`GloveModel(rig, hand_kwargs={"glove_port": 4040}, arm_kwargs={...})`,
`HandModel(hand_kwargs={...})`. `GloveModel` takes its arm target from either transport —
`GloveModel(rig, arm_source="quest")` (OSC/UDP, default) or `arm_source="webxr"`
(browser WebXR over WebSocket, no APK to sideload). The WebXR path defaults to
wireless: it serves HTTPS with a self-signed cert on this machine's LAN IP, since
WebXR needs a secure context. Pass `tls=False` for the wired `adb reverse` route.
See `docs/API.md`. Both transports can be installed side by side. For a custom hardware combination, subclass `TeleopModel`
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

- [**Real-robot quickstart**](docs/QUICKSTART.md) (Korean) — clone to running
  teleop on a nero arm + ORCA hand, and what usually goes wrong.
- [**Guide**](docs/GUIDE.md) — bringing up a new robot, calibration workflow, arm-IK
  tuning and how to judge a change, diagnostics, sending to real hardware.
- [**API reference**](docs/API.md) — public symbols per subpackage, with signatures.
- [**Changelog**](CHANGELOG.md) — version history. **0.2.0 contains breaking changes**
  (`model.ik[s]` → `model.sides[s].ik`, `RobotModel.solve` removed).

## Acknowledgments

whatslab builds on excellent open-source work:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (rigid-body kinematics/IK),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting) (DexPilot hand retargeting, MIT),
[viser](https://github.com/nerfstudio-project/viser) (web 3D visualization), and
[LeRobot](https://github.com/huggingface/lerobot) (dataset format).

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0) — source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
