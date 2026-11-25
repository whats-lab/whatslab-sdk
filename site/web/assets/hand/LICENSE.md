# Meta OpenXR hand model — license notice

`left.glb`, `right.glb` in this directory are converted from `OpenXRLeftHand.fbx` /
`OpenXRRightHand.fbx` of the **Meta XR Core SDK** (`com.meta.xr.sdk.core@74.0.0`,
`Meshes/HandTracking/`), skeleton = OpenXR hand joints (`XRHand_*`).

Copyright (c) Meta Platforms, Inc. and affiliates. All rights reserved.

Licensed under the **Oculus SDK License Agreement**:
https://developer.oculus.com/licenses/oculussdk/

This is **not** a Creative Commons asset. Redistribution terms are those of the Oculus SDK
License Agreement; check them before shipping these files outside the team.

## Changes made

- Converted FBX → glTF binary (`.glb`) with `tools/meta_fbx_to_glb.py` (assimp export).
- Unit scale 0.01 on the root node (cm → m).
- Mesh, skin weights, bone hierarchy and bind poses are otherwise unmodified.

The same mesh and bind pose also exist as the CC BY 4.0 Oculus Hand Models 1.0
(`OculusHand_{L,R}.fbx`, legacy `b_{l,r}_*` skeleton); `tools/meta_fbx_to_glb.py` converts only the OpenXR
(`XRHand_*`) FBX used here.

# Bone meshes — `left_bones.glb`, `right_bones.glb`

The node hierarchy, bind poses and skin of these files are taken from the Meta OpenXR hand above (same terms),
plus one extra node `XRHand_Trapezium` added by `tools/bake_bones.py`.

The bone meshes come from **BodyParts3D**:

> BodyParts3D, © The Database Center for Life Science licensed under CC Attribution 4.0 International

- License: Creative Commons Attribution 4.0 International — https://creativecommons.org/licenses/by/4.0/
  (BodyParts3D license page: https://dbarchive.biosciencedbc.jp/en/bodyparts3d/lic.html, updated 2025-02-27).
- Carpals and trapezium: BodyParts3D Release 4.0 meshes in `hand/bones_src/bodyparts3d/` (unmodified sources).
- Phalanges and metacarpals: the project's earlier hand bone STL meshes (git `085bbd3^`, `hand/meshes`), which
  were made from BodyParts3D.
- Changes: rigidly aligned, scaled and fitted inside the Meta hand skin, re-skinned to the `XRHand_*` skeleton
  (`tools/bake_bones.py`).
