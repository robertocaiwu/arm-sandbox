# arm_sandbox_viz

`rerun_bridge` (C++) shows the running sim in the browser through Rerun. It subscribes only to
standard topics (FR-14a), so any viewer can replace it (M12) without touching the rest.

## How the robot moves in Rerun

Rerun 0.38 models transforms like TF: named coordinate frames connected by parent→child transforms.
The URDF loader puts each link's meshes in a frame named after the link and logs the joint transforms
at zero. The bridge then logs every `/tf` transform with the same parent/child frame names, timed by
the message stamp on the `sim_time` timeline; the newest transform wins, so the meshes follow the
robot. `/joint_states` become time-series plots.

Config: `config/rerun_bridge.yaml` (`grpc_url` to stream, or `save_path` to record).
The Rerun C++ SDK is installed by `scripts/install_deps.sh` (D17).
