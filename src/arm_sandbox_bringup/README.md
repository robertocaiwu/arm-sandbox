# arm_sandbox_bringup

Top-level launch files and per-robot controller configs.

    make start_sim                                  # = ros2 launch arm_sandbox_bringup sim.launch.py
    make start_sim ARGS="viewer:=false gravcomp:=true rerun:=false"

Arguments: `robot` (default `panda`), `viewer` (native MuJoCo window), `gravcomp` (D15), `rerun`
(web viewer at http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy), `rerun_save` (record to a `.rrd` file instead).

## Control

`arm_controller` is the stock Joint Trajectory Controller on the `effort` interface. It interpolates
the trajectory and closes a PID loop per joint, τ = Kp·e + Ki·∫e + Kd·ė. Without simulator gravity
compensation, the integral term carries the gravity load, so tracking settles within a second rather
than instantly; the M3 controllers add g(q) explicitly. Gains are in `config/panda_controllers.yaml`.

## Startup order

JTC holds the pose it sees when it activates, and an unpowered arm falls within a fraction of a
second. The launch therefore loads the arm and gripper controllers inactive, and `activate_at_home`
resets the sim to the `home` keyframe and activates them in back-to-back service calls.
