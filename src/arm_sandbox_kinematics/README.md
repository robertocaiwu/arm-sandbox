# arm_sandbox_kinematics

Hand-written kinematics for serial arms, in plain C++17 + Eigen (no ROS). Built from a URDF, so it
works for any arm. Checked against Pinocchio in the tests (FR-18).

## Forward kinematics

T(q) = Π_i O_i · M_i(q_i) · T_tip. O_i is joint i's fixed origin (fixed joints before it folded in),
M_i rotates by q_i about the joint axis (revolute) or translates along it (prismatic).

## Geometric Jacobian

Maps joint velocities to the tip's twist in the base frame, [v; ω] = J(q) q̇. Revolute column:
[z_i × (p_tip − p_i); z_i], prismatic: [z_i; 0], with z_i and p_i the joint's world axis and origin.
Manipulability w = √det(J Jᵀ) measures how far the arm is from a singularity (w = 0).

## Inverse kinematics (damped least squares)

Iterate Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e on the pose error e = [p* − p; θ·axis(R* Rᵀ)].
- λ = 0 away from singularities (Gauss–Newton, a few iterations); below the manipulability
  threshold w₀, λ² = λ_max² (1 − (w/w₀)²) keeps steps bounded where the pseudo-inverse explodes.
- A 7-DoF arm has one redundant degree of freedom: the null-space term k · N (q_home − q), with
  N = I − V_r V_rᵀ from the SVD of J, moves the joints towards home without moving the tip.
- Joint clamping: a joint at its limit that the step pushes further out is dropped from J, and the
  step is solved again.

A local solver started far away can miss solutions that need a different arm configuration
(elbow flipped); seeding from the current pose, as the reach runner does, avoids that.
