# arm_sandbox_sim

What arm-sandbox adds on top of `mujoco_ros2_control` (D13). For now: composing the MuJoCo scene at
launch (`arm_sandbox_sim.scene`). Later: seeded randomized reset and ground truth (M4).

## Gravity compensation (D15)

A torque-controlled arm falls unless something cancels gravity, τ = g(q). Either the controller
computes g(q) (default here, `gravcomp:=false`), or the simulator applies it as a passive force
(`gravcomp:=true`, MuJoCo's body `gravcomp`), which is what the real Panda does inside its torque
interface. Only the robot's bodies are compensated, so task objects keep their weight.

The module is ROS-free so the Phase B Gymnasium environment loads exactly the same model.
