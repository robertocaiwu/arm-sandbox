# Franka Emika Panda Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 2.3.3 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a simplified robot description (MJCF) of the [Franka Emika
Panda](https://www.franka.de/) developed by [Franka
Emika](https://www.franka.de/company). It is derived from the [publicly
available URDF
description](https://github.com/frankaemika/franka_ros/tree/develop/franka_description).

<p float="left">
  <img src="panda.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Converted the DAE [mesh
   files](https://github.com/frankaemika/franka_ros/tree/develop/franka_description/meshes/visual)
   to OBJ format using [Blender](https://www.blender.org/).
2. Processed `.obj` files with [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf).
3. Eliminated the perfectly flat `link0_6` from the resulting submeshes created for `link0`.
4. Created a convex decomposition of the STL collision [mesh
   file](https://github.com/frankaemika/franka_ros/tree/develop/franka_description/meshes/collision)
   for `link5` using [V-HACD](https://github.com/kmammou/v-hacd).
5. Added `<mujoco> <compiler discardvisual="false"/> </mujoco>` to the
   [URDF](https://github.com/frankaemika/franka_ros/tree/develop/franka_description/robots)'s
   `<robot>` clause in order to preserve visual geometries.
6. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
7. Matched inertial parameters with [inertial.yaml](
   https://github.com/frankaemika/franka_ros/blob/develop/franka_description/robots/common/inertial.yaml).
8. Added a tracking light to the base.
9. Manually edited the MJCF to extract common properties into the `<default>` section.
10. Added `<exclude>` clauses to prevent collisions between `link7` and `link8`.
11. Manually designed collision geoms for the fingertips.
12. Added position-controlled actuators for the arm.
13. Added an equality constraint so that the left finger mimics the position of the right finger.
14. Added a tendon to split the force equally between both fingers and a
    position actuator acting on this tendon.
15. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

### MJX

A version of the Franka Emika Panda environment was created for MJX. Steps:

1. Added `mjx_panda.xml`, forked from `panda.xml`.
2. Added `mjx_scene.xml` and `mjx_single_cube.xml`, forked from `scene.xml`.
3. Gripper collision geometries were modified to contain less geoms. A capsule collision geom was added to the hand.
4. Solver parameters were tuned for performance.
5. Actuator `kp` and `kv` were reduced for more stable simulation.
6. Added a `site` to the gripper.
7. Removed tendon and added position actuator for the gripper. Changed gripper `ctrlrange`.

## License

This model is released under an [Apache-2.0 License](LICENSE).

---

## arm-sandbox notes: what the derivation steps mean

> Everything above this line is the upstream MuJoCo Menagerie README (commit in
> [`MENAGERIE_COMMIT`](MENAGERIE_COMMIT)). This section is ours: it explains each derivation
> step and what it means for arm-sandbox.

### 1. Meshes (steps 1–4)

- **DAE → OBJ (step 1).** Franka's visual meshes are COLLADA (`.dae`), which MuJoCo can't load.
  It loads OBJ and STL.
- **`obj2mjcf` (step 2).** MuJoCo allows one material per mesh, so a multi-colored part like
  `link0` is split into one submesh per material (`link0_0.obj` … `link0_11.obj`). Each becomes its
  own `<geom>` with its own color.
- **Drop `link0_6` (step 3).** A perfectly flat submesh has zero volume. MuJoCo rejects
  degenerate meshes or computes nonsense inertia for them.
- **V-HACD on `link5` (step 4).** MuJoCo collides every mesh using its **convex hull**. `link5`
  is strongly non-convex, so its hull would fill in the concave parts and block motions the real
  arm can make. V-HACD splits it into three convex pieces (`link5_collision_{0,1,2}.obj`) whose
  union fits the real shape. The other links are close enough to convex to use their STL directly.

### 2. Automatic conversion (steps 5–6)

- **`discardvisual="false"` (step 5).** By default, MuJoCo's URDF importer drops `<visual>`
  elements and keeps only collision geometry. This flag keeps the visual meshes.
- **URDF → MJCF (step 6).** MuJoCo loads a URDF directly, and saving the loaded model
  (`mj_saveLastXML`) produces the first MJCF. Everything after this is manual cleanup.

### 3. Physics fixes (steps 7, 10)

- **Inertials (step 7).** Franka's URDF ships with placeholder inertias. The real mass, center of
  mass, and full inertia tensor per link come from `inertial.yaml`. Gravity compensation,
  computed torque, and OSC all depend on these numbers.
- **Contact exclusion (step 10).** MuJoCo already ignores contact between a parent body and its
  child, *unless the parent is welded to the world*. `link0` has no joint, so it counts as part
  of the world, and its contact with `link1` has to be excluded explicitly (`<contact>` at the end
  of `panda.xml`). Without it, the overlapping meshes at the base would produce constant spurious
  contact forces. The upstream README still says "`link7` and `link8`": URDF's `link8` is a fixed
  frame that the importer merged into `link7`, and the current model excludes `link0`/`link1`
  ([CHANGELOG.md](CHANGELOG.md)).

### 4. Cleanup and cosmetics (steps 8, 9, 11, 15)

- **Tracking light (step 8).** A `trackcom` light follows the robot's center of mass, so the arm
  stays lit in the viewer.
- **Defaults (step 9).** Repeated properties live in `<default>` classes (`panda`, `visual`,
  `collision`, `finger`), so each body only states what's unusual about it. `visual` geoms have
  `contype=0 conaffinity=0` (rendered, never collide) and sit in group 2. `collision` geoms sit in
  group 3, so each set can be toggled in the viewer.
- **Fingertip pads (step 11).** The finger meshes give poor contact for grasping. Five small
  boxes per finger (`fingertip_pad_collision_1..5`) give flat, stable contact patches.
- **Scene (step 15).** `scene.xml` adds a floor, a skybox, and haze, so the robot file itself
  stays free of any scene.

### 5. Actuation (steps 12–14)

**Arm (step 12).** Each joint has a `general` actuator with `biastype="affine"`. MuJoCo computes
its force as

```
force = gain·ctrl + b0 + b1·q + b2·q̇
      = kp·ctrl − kp·q − kv·q̇            (gainprm = kp, biasprm = "0 −kp −kv")
      = kp·(q_target − q) − kv·q̇
```

So each joint is a **PD position servo** (kp = 4500 / 3500 / 2000, kv = kp/10), with the force
capped by `forcerange` at the Panda's torque limits (±87 Nm, or ±12 Nm on joints 5–7).

**Gripper (steps 13–14).** Two slide joints driven by one motor:

- The fixed **tendon** `split` has length L = 0.5·q₁ + 0.5·q₂ (the average finger opening). One
  actuator pulls on L, and the tendon spreads its force equally to both fingers.
- The **equality constraint** keeps q₁ = q₂, so the fingers move symmetrically, like the real
  gripper's mechanical coupling.
- The actuator is affine too: force = 0.0157·ctrl − 100·L − 10·L̇. At rest, L = 0.0157·ctrl / 100,
  so ctrl = 255 gives L = 0.04 m (fully open). The 0–255 range copies the Robotiq-style command
  convention.

### MJX

The MJX steps describe a separate fork for JAX/GPU training (`mjx_*.xml`). Those files are not
vendored here.

### Changes arm-sandbox still needs (M1)

1. **Torque actuators.** The design spec makes every arm controller write the `effort` command
   interface. The step-12 PD servos must become `motor` actuators (keeping `forcerange` as the
   torque limit). Otherwise our controllers' effort commands would be stacked on top of MuJoCo's
   built-in PD loop.
2. **Joint names.** This model uses `joint1`…`joint7` and `finger_joint1/2`. The upstream URDF
   and the planned `robot.yaml` use `panda_joint1`… and `panda_finger_joint1`. The names must
   match for the `ros2_control` mapping and the URDF/MJCF FK agreement test.
