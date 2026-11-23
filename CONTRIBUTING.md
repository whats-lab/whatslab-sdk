<p align="center"><b>English</b> | <a href="CONTRIBUTING.ko.md">한국어</a></p>

# Contributing

Rules to read before changing whatslab-sdk, and the reasons behind them. Installation and usage are in
the [README](README.md); public symbols are in [API.md](docs/API.md) (Korean).

## Scope

The SDK covers **the hand** only: hand input, hand retargeting, a joint safety filter and visualization.

- No arm IK, wrist pose or arm control. The caller gets the wrist pose from its own tracker.
- No data recording. The SDK hands out plain data (OpenXR 26 joints, `{joint: rad}`, timestamps) and the
  caller picks the storage format.
- Before adding something, ask whether the caller could do it instead. Widening the scope is what made
  the SDK complex up to 0.4.

## The SDK is canonical

This repository defines the API. Repositories that use the SDK (whatslab-ros2 and others) adapt to it.
Do not keep compatibility code, options or aliases in the SDK for one consumer. For a breaking change,
mark the commit title with `!` and fix the consumers.

## Structure

```
core      ← receiver
core      ← retarget
receiver, retarget ← hand.py
```

- `core` imports no other subpackage.
- `receiver` and `retarget` do not know each other. Only `InputSample` and `HandPose` from
  `core.types` connect them.
- Every input arrives as an OpenXR 26-joint `(26, 8)` array (`HandPose.xr_joints`, wrist-relative).
  A new input device converts to this format.
- Heavy optional dependencies (dex-retargeting, torch, pin, viser) are imported only by the modules
  that use them. The GeoRT path of `whatslab.retarget` must import with numpy alone.

### Retargeting engines

`HandRetargeter` is the entry point; engines only compute. The engine contract is the `Engine` Protocol
in `retarget/retargeter.py` (structural, no base class); a test checks each engine against it.

| An engine provides | `HandRetargeter` handles |
|---|---|
| `solve(xr_joints) -> q`, `joint_names`, `joint_limits`, `reset()` | picking the engine from the prefix (`dex:`, `geort:`) |
| raises a `HandInputError` subclass for input it cannot solve | `reset()` on tracking loss; holding the last command on unsolvable input |

Each engine keeps its own input preprocessing in `<engine>/preprocess.py` (OpenXR 26 joints → that
engine's input). A new engine must not re-implement tracking-loss handling or last-value holding.

## Code rules

- **No docstrings or comments.** Make names and structure carry the meaning; put explanations in the
  README or API.md. The same holds for pyproject and the config YAML. (Tool directives such as `# noqa`
  are the exception.)
- **No try/except.** Catching and passing on exceptions hides bugs; let failures reach the caller. The
  one exception today is `HandRetargeter.compute` catching `HandInputError` to hold the last command —
  keeping the robot hand from jumping on one bad frame is the feature itself.
- No defensive code: no re-checking on already validated paths, no handling of cases that cannot happen.
- No single-use wrappers, classes or options. No `X = Y` aliases and no `import … as …` (except
  conventions such as np).
- Names: engines are `…Engine`, receivers `…Receiver`; an engine computes with `solve`, the entry point
  with `compute`. The human-hand side argument is `side`.
- PEP 8 is checked with ruff (pycodestyle, pep8-naming, isort, pyflakes; line length 99). Public APIs
  carry type hints.

## Development

```bash
pip install -e '.[all,dev]'
ruff check .
pytest -q
```

- Changes come with tests. A bug fix starts with a test that reproduces the bug.
- The full test run occasionally ends in a segfault. It is a known issue in the path dex-retargeting
  uses to read URDFs (yourdfpy → pytransform3d), not in this repository; re-run and it passes.
- On hardware: the glove with `examples/airglove_hand.py --viz`; Quest hand tracking by passing a
  `WebXRHandReceiver` to `HandModel(hand_source=…)` and opening the printed URL in the headset browser.

## Commits and releases

- Commit title: `type(scope): what` — type is `feat`, `fix`, `refactor`, `docs`, `chore` or `test`.
  Add `!` for an API break (`refactor!: …`). The body says why.
- No CHANGELOG. Commits and PRs are the change record.
- The version lives only in `pyproject.toml`; a release is a `vX.Y.Z` tag, and consumers pin the tag.
- Not uploaded to PyPI (the `Private :: Do Not Upload` classifier blocks it).
- Update the docs in the same commit. When a public name or signature changes, check API.md, both
  README languages and QUICKSTART.
