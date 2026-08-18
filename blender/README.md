# Blender -> Unreal sword prop

Scratch workspace for the sword prop. Nothing here affects the Sominus site,
which is served from `main`; this folder only exists on the
`claude/blender-replica-geometry-v31xrg` branch.

## Running it

From the Blender GUI: **Scripting** tab -> **Open** -> `sominus_sword.py` ->
**Run Script** (Alt+P).

Headless:

    blender --background --python sominus_sword.py -- --out ./export

Writes `SM_SominusSword.fbx`, `.glb` and `.blend` into the output folder.

## What it already handles

- Real-world scale, so the FBX imports at 1 uu = 1 cm with Unreal's defaults
- Origin at the grip, so socketed rotation behaves
- Sharp edges marked by angle, exported as smoothing groups (`mesh_smooth_type='EDGE'`)
- UV unwrap, with a planar fallback if `smart_project` is unavailable in background mode
- Three material slots: blade / fittings / grip
- `UCX_` convex collision hulls, parented to the mesh and included in the export

## What still needs the reference

The silhouette. Everything shape-related lives in the `PARAMS` block at the top
of the script and in the profile tables just below it.
