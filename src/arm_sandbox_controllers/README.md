# arm_sandbox_controllers

Task-space controllers for torque-controlled arms: a ROS-free control core and two `ros2_control`
plugins on top of it. Select one with `make start_sim ARGS="arm_controller:=osc_controller"` (or
`cartesian_impedance_controller`), then publish a `geometry_msgs/PoseStamped` to `~/target_pose`.

## The model

Joint-space dynamics M(q) q'' + C(q, q') q' + g(q) = tau come from Pinocchio on the URDF, reduced to
the arm joints, plus the actuators' armature (reflected rotor inertia), which the MJCF has and the
URDF can't express. Kinematics (J, FK) come from arm_sandbox_kinematics.

## The laws

Both work on the end-effector error e = [p* - p; angle-axis(R* R^T)] and twist x' = J q':

    tau = J^T F + N^T tau_0 + C q' (+ g)        Lambda = (J M^-1 J^T)^-1,  N^T = I - J^T Jbar^T

- Operational space (Khatib): F = Lambda (Kp e - Kd x'). Lambda turns the end effector into a unit
  mass in every direction, so the gains give the same, decoupled response everywhere: stiff tracking.
- Cartesian impedance (Hogan): F = K e - D x'. A spring-damper: a push F moves the end effector by
  about K^-1 F (test_impedance_compliance.py: 10 N, 500 N/m, 2 cm). Right for contact.
- tau_0 pulls the redundant joint towards the rest pose without moving the end effector (N^T is the
  dynamically consistent null-space projector).
- g is added only when nothing else compensates gravity (`compensate_gravity`, set by the launch).

## Real time

`update()` never allocates: preallocated workspace, allocation-free kinematics overloads, and tests
that run `compute()` under Eigen's EIGEN_RUNTIME_NO_MALLOC guard.
