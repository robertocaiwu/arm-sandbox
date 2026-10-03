# Learning Resources

Background reading for the theory behind each plan, so the code can be understood (and explained)
rather than just run. One section per plan; add a new one when a plan introduces new theory.

> These references were collected from memory. The books and reports are standard and easy to find,
> but check exact chapter, section and page numbers against your own copy.

**Notation warning.** This project writes twists, Jacobians and pose errors with the **linear part
first**, [v; ω], like Siciliano and Pinocchio. *Modern Robotics* puts the **angular part first**,
[ω; v]. Swap the blocks when comparing formulas across sources.

---

## Plan 03 — Kinematics (M2) and the Reach Task

Plan: [plan/2026-10-02-plan-03-kinematics-and-reach.md](plan/2026-10-02-plan-03-kinematics-and-reach.md)

### Two textbooks that cover nearly everything

- **Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*** (free PDF and lecture
  videos at modernrobotics.org, plus a Coursera specialization).
  - Ch. 3: rigid-body motions.
  - Ch. 4: forward kinematics.
  - Ch. 5: velocity kinematics and statics (Jacobian, singularities, manipulability).
  - Ch. 6: inverse kinematics, including Newton–Raphson IK, which is close to what `solve_ik` does.

  The short videos (Kevin Lynch, YouTube) are the gentlest start. It writes FK as a "product of
  exponentials" rather than our URDF-style chain of transforms; the ideas carry over directly.
- **Siciliano, Sciavicco, Villani & Oriolo, *Robotics: Modelling, Planning and Control*** (Springer).
  - Ch. 2: kinematics.
  - Ch. 3: differential kinematics. It covers the geometric Jacobian in exactly our form, plus
    singularities, redundancy and null-space motion, and the classic IK algorithms.

  Closest match to the plan's notation.

### Task 1 — FK, Jacobian, manipulability (`KinematicChain`)

- **URDF joint spec**, wiki.ros.org/urdf/XML/joint. It explains `origin`, `axis` and `limit`, which
  `kinematic_chain.cpp` parses, folding fixed joints into the next moving joint.
- **Yoshikawa (1985)**, "Manipulability of Robotic Mechanisms", *International Journal of Robotics
  Research*. The origin of w = √det(J Jᵀ).
- **Pinocchio documentation**, stack-of-tasks.github.io/pinocchio. Read the pages on frames and
  Jacobians, especially the reference frames `LOCAL`, `WORLD` and `LOCAL_WORLD_ALIGNED`. That's why
  `test_kinematic_chain.cpp` compares against `LOCAL_WORLD_ALIGNED`.
- **Eigen Geometry module docs** (`Isometry3d`, `AngleAxisd`, quaternions). They explain the
  orientation error θ·axis used in the IK and the tests.

### Task 2 — Inverse kinematics by damped least squares (`solve_ik`)

- **Start here: Samuel Buss (2004)**, "Introduction to Inverse Kinematics with Jacobian Transpose,
  Pseudoinverse and Damped Least Squares methods" (UCSD technical report, free PDF). Short and
  clear; compares exactly the methods we chose between.
- **Nakamura & Hanafusa (1986)**, "Inverse Kinematic Solutions with Singularity Robustness for Robot
  Manipulator Control", *Journal of Dynamic Systems, Measurement, and Control*. The origin of damped
  least squares and of damping that grows as manipulability drops (our `max_damping` and
  `manipulability_threshold`).
- **Chiaverini, Siciliano & Egeland (1994)**, "Review of the Damped Least-Squares Inverse Kinematics
  with Experiments on an Industrial Robot Manipulator", *IEEE Transactions on Control Systems
  Technology*. Compares damping strategies in practice.
- **Baerlocher & Boulic (2004)**, "An inverse kinematics architecture enforcing an arbitrary number of
  strict priority levels", *The Visual Computer*. Joint clamping (drop a joint stuck at its limit and
  solve again) and prioritized null-space tasks (our pull towards home).
- **Null-space motion for redundant arms** (the Panda has 7 joints for a 6-D pose): Siciliano Ch. 3
  (redundancy), or *Modern Robotics* Ch. 6.
- **Pinocchio's inverse-kinematics example** in its docs: a short damped pseudo-inverse loop in Python,
  useful to compare line by line with `ik.cpp`.

### Tasks 3–4 — The reach task and the ROS node (`reach_runner`)

- **control.ros.org, Joint Trajectory Controller**: goal tolerances, and the PID gains on the effort
  interface we use.
- **ROS 2 Humble docs** (docs.ros.org/en/humble):
  - the tf2 tutorials, for how `reach_runner` reads the end-effector pose;
  - "Writing an action client (C++)", for how it sends `FollowJointTrajectory` goals.
- **`launch_testing` README** in the ROS 2 `launch` repository: `ReadyToTest`, `post_shutdown_test`,
  `proc_info`, used by the end-to-end tests.

### Suggested order

1. *Modern Robotics* Ch. 4–6 videos, for intuition.
2. Buss's report, for the IK method itself.
3. `src/arm_sandbox_kinematics/src/kinematic_chain.cpp` and `src/ik.cpp` side by side with
   Siciliano Ch. 3. The code comments point at the same formulas.
4. Plan 03's theory blurbs, then `src/arm_sandbox_kinematics/README.md` (added in Plan 03, Task 5)
   for a one-page summary.
