<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%" ></p>

<h1 align="center">whatslab</h1>

<p align="center"><b>English</b> | <a href="README.ko.md">한국어</a></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.10%2B-blue.svg" alt="Python"/>
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
- [Package layout](#package-layout)
- [Examples & tools](#examples--tools)
- [Documentation](#documentation)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## Main features

- One hand input for every source: the OpenXR 26-joint hand (`HandPose.xr_joints`), from the
  AirGlove (through Spine) or from Quest hand tracking (headset browser, WebXR)
- Two retargeting engines:
  - DexPilot vector retargeting (numpy port of
    [dex-retargeting](https://github.com/dexsuite/dex-retargeting), no torch) plus a per-frame
    posture prior from the human joint angles (`posture_prior`, on by default), 1–2 ms per hand
    per frame, any robot hand with a URDF
  - the **GeoRT soft-prompt** network: one forward pass in numpy, about 0.2 ms, one trained
    model shared by several robot hands
- Output is `{side: {joint_name: rad}}`, ready to publish; an optional `SafetyFilter` clamps
  position and rate
- Pure Python, no ROS dependency; strict dependency direction
  (`receiver → core`, `retarget → core`, `hand → core·receiver·retarget`)

### Supported hands

**DexPilot** — `dex:<name>`, where `<name>` is a file in `configs/hands/` (`dex:orca` resolves
to `orca_{side}`; a file path also works). A config for one side serves the other side by
mirroring the input.

Configs for every hand in `dexhand-description` (human-shaped hands with 4 or 5 fingers), grouped as in
that repository. Every hand is previewed with DexPilot retargeting in the browser at
**[whats-lab.github.io/whatslab-sdk](https://whats-lab.github.io/whatslab-sdk/)**: synthetic motions (fist, pinches, V pose)
retargeted offline next to a human hand. Meshes are downloaded by the browser from each hand's upstream
repository at the pinned commit.

**Korean hands**

| Hand | Engine name | Config | Notes |
|---|---|---|---|
| Tesollo DG-5F-S (20-DoF) | `dex:dg5f_s` | `dg5f_s_{left,right}.yaml` | 5 fingers, 20 actuated; PIP/DIP limited to −5..90° |
| Tesollo DG-5F-S 15-DoF | `dex:dg5f_s_15dof` | `dg5f_s_15dof_{left,right}.yaml` | 5 fingers, 15 actuated; PIP/DIP limited to −5..90° |
| Tesollo DG-5F | `dex:dg5f` | `dg5f_{left,right}.yaml` | 5 fingers, 20 actuated; PIP/DIP limited to −5..90° |
| ROBOTIS HX5-D20 | `dex:robotis` | `robotis_{left,right}.yaml` | 5 fingers, 20 actuated; fingertip sensor offsets |
| Allegro Hand | `dex:allegro` | `allegro_{left,right}.yaml` | 4 fingers, 16 actuated; human little finger not used |
| Allegro Hand V5 (4 finger, A) | `dex:allegro_v5_4f` | `allegro_v5_4f_{left,right}.yaml` | 4 fingers, 16 actuated; wrist offset |
| Allegro Hand V5 Sense | `dex:allegro_v5_sense` | `allegro_v5_sense_{left,right}.yaml` | 4 fingers, 16 actuated; wrist offset |
| Allegro Hand V6 | `dex:allegro_v6` | `allegro_v6_{left,right}.yaml` | 5 fingers, 20 actuated; tip offsets on distal links |
| AIDIN Hand Gen2 | `dex:aidin_hand2` | `aidin_hand2_{left,right}.yaml` | 5 fingers, 16 actuated; each joint4 is passive (four-bar from joint3) — approximated as a linear coupling (`couplings`, least-squares fit to the upstream four-bar table, max error 6.3° fingers / 8.7° thumb); tactile pad links kept; tip offsets on distal links |

**Korean humanoid hands**

| Hand | Engine name | Config | Notes |
|---|---|---|---|
| ROBROS IGRIS-C hand | `dex:igris_c` | `igris_c_{left,right}.yaml` | 5 fingers, 6 actuated; distal joints passive — approximated as 1:1 coupled to the middle joint (`couplings`, real ratio unconfirmed, to be checked with ROBROS); tip offsets on distal links |

**Foreign hands**

| Hand | Engine name | Config | Notes |
|---|---|---|---|
| ORCA Hand v2 | `dex:orca` | `orca_{left,right}.yaml` | 5 fingers, 16 actuated; wrist joint held at 0; tip offsets on distal links |
| ORCA Hand v1 | `dex:orca_v1` | `orca_v1_{left,right}.yaml` | 5 fingers, 16 actuated; wrist joint held at 0 |
| LEAP Hand | `dex:leap` | `leap_{left,right}.yaml` | 4 fingers, 16 actuated; human little finger not used |
| LEAP Hand V2 Advanced | `dex:leap_hand_v2_adv` | `leap_hand_v2_adv_{left,right}.yaml` | 5 fingers, 21 actuated; wrist offset |
| PSYONIC Ability Hand | `dex:ability` | `ability_{left,right}.yaml` | 5 fingers, 6 actuated; 4 mimic |
| Schunk SVH | `dex:schunk` | `schunk_{left,right}.yaml` | 5 fingers, 9 actuated; 11 mimic |
| Shadow Hand | `dex:shadow` | `shadow_{left,right}.yaml` | 5 fingers, 22 actuated; wrist joints held at 0 |
| Inspire Hand | `dex:inspire` | `inspire_{left,right}.yaml` | 5 fingers, 6 actuated; 6 mimic |
| LinkerHand L6 | `dex:linkerhand_l6` | `linkerhand_l6_{left,right}.yaml` | 5 fingers, 6 actuated; 5 mimic |
| LinkerHand O6 | `dex:linkerhand_o6` | `linkerhand_o6_{left,right}.yaml` | 5 fingers, 6 actuated; 5 mimic; tip offsets on distal links |
| LinkerHand L20 | `dex:linkerhand_l20` | `linkerhand_l20_{left,right}.yaml` | 5 fingers, 16 actuated; 5 mimic; tip offsets on distal links |
| LinkerHand L20 Lite | `dex:linkerhand_l20lite` | `linkerhand_l20lite_{left,right}.yaml` | 5 fingers, 10 actuated; 10 mimic; tip offsets on distal links |
| LinkerHand O7 | `dex:linkerhand_o7` | `linkerhand_o7_{left,right}.yaml` | 5 fingers, 7 actuated; 10 mimic; tip offsets on distal links |
| LinkerHand L25 | `dex:linkerhand_l25` | `linkerhand_l25_{left,right}.yaml` | 5 fingers, 16 actuated; 5 mimic; tip offsets on distal links |
| LinkerHand O20 | `dex:linkerhand_o20` | `linkerhand_o20_{left,right}.yaml` | 5 fingers, 16 actuated; tip offsets on distal links |
| LinkerHand L30 | `dex:linkerhand_l30_right` | `linkerhand_l30_right.yaml` | 5 fingers, 20 actuated; tip offsets on distal links; one URDF; the other side is mirrored |
| LinkerHand O30i | `dex:linkerhand_o30` | `linkerhand_o30_{left,right}.yaml` | 5 fingers, 20 actuated; tip offsets on distal links |
| LinkerHand T12 | `dex:linkerhand_t12_right` | `linkerhand_t12_right.yaml` | 5 fingers, 14 actuated; 5 mimic; tip offsets on distal links; one URDF; the other side is mirrored |
| BrainCo Revo2 | `dex:brainco_revo2` | `brainco_revo2_{left,right}.yaml` | 5 fingers, 6 actuated; 5 mimic |
| BrainCo Revo3 | `dex:brainco_revo3` | `brainco_revo3_{left,right}.yaml` | 5 fingers, 21 actuated |
| DexRobot DexHand021 | `dex:dexrobot_dexhand021` | `dexrobot_dexhand021_{left,right}.yaml` | 5 fingers, 20 actuated |
| Sharpa Wave | `dex:sharpa_wave` | `sharpa_wave_{left,right}.yaml` | 5 fingers, 22 actuated |
| Wuji Hand | `dex:wuji_hand` | `wuji_hand_{left,right}.yaml` | 5 fingers, 20 actuated |
| Wuji Hand 2 (2.1) | `dex:wuji_hand2` | `wuji_hand2_{left,right}.yaml` | 5 fingers, 20 actuated; URDF root is the arm-flange mount, wrist link `r_wrist` |
| Rysen Apex Hand | `dex:rysen_apex_hand` | `rysen_apex_hand_{left,right}.yaml` | 5 fingers, 16 actuated; 5 mimic |
| DexHand (TheRobotStudio) | `dex:dexhand_trs` | `dexhand_trs_{left,right}.yaml` | 5 fingers, 21 actuated; wrist joints held at 0 |

**Foreign humanoid hands**

| Hand | Engine name | Config | Notes |
|---|---|---|---|
| Inspire RH56DFTP (Unitree G1 model) | `dex:inspire_rh56dftp_unitree` | `inspire_rh56dftp_unitree_{left,right}.yaml` | 5 fingers, 6 actuated; 6 mimic; tip offsets on distal links |
| Unitree Dex5-1 | `dex:unitree_dex5_1` | `unitree_dex5_1_{left,right}.yaml` | 5 fingers, 20 actuated; tip offsets on distal links |
| Unitree Dex5-S | `dex:unitree_dex5_s` | `unitree_dex5_s.yaml` | 5 fingers, 22 actuated; tip offsets on distal links; one URDF; the other side is mirrored |
| Unitree Dex2-5 | `dex:unitree_dex2_5` | `unitree_dex2_5_{left,right}.yaml` | 5 fingers, 10 actuated; tip offsets on distal links |
| Fourier Hand 6-DoF | `dex:fourier_hand_6dof` | `fourier_hand_6dof_{left,right}.yaml` | 5 fingers, 6 actuated; 5 mimic |
| RobotEra XHAND1 (STAR1 model) | `dex:robotera_xhand1_star1` | `robotera_xhand1_star1_{left,right}.yaml` | 5 fingers, 12 actuated |
| RobotEra XHAND1 Pro U | `dex:robotera_xhand1_pro_u` | `robotera_xhand1_pro_u_{left,right}.yaml` | 5 fingers, 20 actuated |
| AgiBot OmniHand 2025 | `dex:agibot_omnihand_2025` | `agibot_omnihand_2025_{left,right}.yaml` | 5 fingers, 10 actuated; 6 mimic |

**GeoRT soft prompt** — `geort:<name>`, where `<name>` is a file in `configs/geort/` (a path
ending in `.yaml` also works). The trained model ships with the package
(`retarget/geort/assets/geort_prompt.npz`, about 5 MB). The model file is looked up in this order:
the config's `model:`, then `$WHATSLAB_GEORT_MODEL`, then the bundled file. The model is
right-handed; a left hand is mirrored onto it.

The keys of both config formats are described in [API.md](docs/API.md).

| Hand | Engine name | URDF in `dexhand-description` |
|---|---|---|
| ORCA Hand v2 (right) | `geort:orca_right` | `orca_hand_v2/orca_hand_v2_right.urdf` |
| ROBOTIS HX5-D20 (right) | `geort:hx5_right` | `robotis_hx5_d20/urdf/hx5_d20_right.urdf` |

The model also holds Allegro and DG-5F-S prompts, trained against URDFs that are not in
dexhand-description yet.

Training: `whatslab-geort-train` (`pip install '.[train]'`) builds the training data from
`configs/geort_train/<hand>.yaml` (42 right hands with 5 or 4 fingers) and trains and exports a model
on any GPU host; see [API.md](docs/API.md). The training code and the bundled model are based on
[facebookresearch/GeoRT](https://github.com/facebookresearch/GeoRT) (CC BY-NC 4.0) and are for
**non-commercial use** only. Human hand-motion data is not included.

## Installation

Published under a source-available license. It is not on PyPI; install from source. The
`Private :: Do Not Upload` classifier makes PyPI reject an accidental upload.

```bash
pip install '.[all]'      # receiver + hand + viz + webxr
pip install '.[receiver]' # or any subset: receiver / hand / viz / webxr
pip install -e '.[all]'   # editable, for development
```

| Extra | Brings in | Needed for |
|---|---|---|
| `receiver` | `airglove-client` (git tag v1.0.0) | AirGlove (Spine) input |
| `webxr` | `websockets`, `cryptography` | Quest hand tracking (headset browser) |
| `hand` | `dexhand-description` | the DexPilot engine (URDF assets) |
| `viz` | `viser`, `trimesh`, `pycollada` | robot-hand visualization |
| `train` | `torch`, `trimesh`, `dexhand-description` | GeoRT model training (`whatslab-geort-train`) |

GeoRT inference (`geort:`) needs only numpy. The DexPilot engine carries its own numpy port of
the dex-retargeting optimizer and its own numpy URDF kinematics (`whatslab.kinematics`), so it
needs neither torch, dex-retargeting nor pinocchio — numpy, scipy (SLSQP) and pyyaml only, which also
makes it pure Python (Python 3.10–3.14, and Pyodide). numpy has no upper bound. `pin` is a `dev`
dependency only: the tests check the kinematics against pinocchio when it is installed.
URDFs and meshes come from
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf) (pulled in by the
`hand` extra); point `WHATSLAB_MODELS_ROOT` at another asset tree to override it. Kinematics needs only
the URDFs; `viz` and GeoRT training download the hand's original meshes once into the user cache
(`dexhand-fetch <hand>` does the same ahead of time, e.g. before going offline).

## Compatibility

| Input | Needs |
|---|---|
| AirGlove | **Spine 3.0.0** (OSC protocol 3) running on the same machine |
| Quest hand tracking | the headset's browser (WebXR); no app to install, no Spine |

`AirGloveReceiver` wraps [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client)
(`airglove-client` 1.0.0). Spine 2.x is not supported since whatslab 1.0.0; use whatslab 0.4.x
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
from whatslab.hand import HandModel

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

Your own hand source: build an `InputSample` and call `HandRetargeter` directly.

```python
from whatslab.core.types import HandPose, InputSample
from whatslab.retarget import HandRetargeter

r = HandRetargeter("geort:orca_right", side="right")
cmd = r.compute(InputSample(hand=HandPose(xr_joints=xr, tracked=True)))   # xr: (26, 8) OpenXR joints
cmd.joint_names, cmd.joint_angles
```

`HandRetargeter` picks the engine from the prefix (`dex:` → `DexEngine`, `geort:` → `GeortEngine`),
resets the engine when tracking is lost and holds the last command when an input cannot be solved.
Both engines expose `solve(xr_joints)` if you want the bare computation.

## Package layout

```
whatslab/
├── core/             types (HandPose, InputSample, HandCommand), Receiver interface, OpenXR joints
├── receiver/
│   ├── glove/        AirGloveReceiver (airglove-client, connection status and alarms)
│   └── webxr/        WebXRHandReceiver, its websockets server, TLS certificate, headset page
├── retarget/
│   ├── retargeter.py HandRetargeter, Engine protocol
│   ├── dex/          engine.py DexEngine (DexPilot) · preprocess.py OpenXR → MANO, palm frame, mirroring
│   └── geort/        engine.py GeortEngine · preprocess.py Kabsch input frame · prompt.py model · bundled npz
│       └── train/    training CLI (torch, not imported by inference) · GeoRT chamfer + license
├── hand.py           HandModel: receiver → retargeter → optional SafetyFilter
├── safety.py         SafetyFilter
├── viz/              HandViz, URDFScene (viser)
├── paths.py          configs / models roots
└── configs/          hands/ (DexPilot, 85 files), geort/ (2 files), geort_train/ (42 files)
```

## Examples & tools

```bash
python examples/airglove_hand.py --config geort:orca_right               # glove → robot hand, prints q
python examples/airglove_hand.py --config dex:orca --viz                 # with the robot hand in viser
python tools/bench_hand_retarget.py --help                               # retargeting quality metrics
```

Run the tests with `pip install -e '.[all,dev]' && pytest`, and the linter with `ruff check .`
(PEP 8 through ruff: pycodestyle, pep8-naming, isort and pyflakes; line length 99).

The hand preview site (`site/`, deployed by `.github/workflows/pages.yml`) builds locally from a
`dexterous-hand-urdf` checkout into `_site/`:

```bash
uv build --wheel --out-dir dist
uv run --no-project --with dist/whatslab_sdk-1.0.0-py3-none-any.whl python site/build.py \
    --repo ../dexterous-hand-urdf --wheel dist/whatslab_sdk-1.0.0-py3-none-any.whl
python -m http.server -d _site
```

## Documentation

- [**Quickstart**](docs/QUICKSTART.md) (Korean) — from clone to a robot hand following the glove,
  and what usually goes wrong
- [**API reference**](docs/API.md) (Korean) — public symbols per subpackage, with signatures
- [**Contributing**](CONTRIBUTING.md) — scope, design rules and code rules for changes

## Acknowledgments

whatslab builds on excellent open-source work:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (reference kinematics in the tests),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting) (DexPilot hand retargeting, MIT —
the DexPilot engine's optimizer is a numpy port of it; see
[THIRD_PARTY_LICENSE](src/whatslab/retarget/dex/THIRD_PARTY_LICENSE)),
[GeoRT](https://github.com/facebookresearch/GeoRT) (geometric hand retargeting, CC BY-NC 4.0 — the
soft-prompt training code and model build on it; its `chamfer_distance` is vendored, see
[THIRD_PARTY_LICENSE_GeoRT.txt](src/whatslab/retarget/geort/train/THIRD_PARTY_LICENSE_GeoRT.txt)),
and [viser](https://github.com/nerfstudio-project/viser) (web 3D visualization).

## License

Licensed under the **주식회사 왓츠랩 (WHATs LAB Corp) Source Code License** (based on
CC BY-NC-ND 4.0): source-available, non-commercial, no derivatives. See [LICENSE](LICENSE).

Copyright © 주식회사 왓츠랩 (WHATs LAB Corp). All rights reserved.
