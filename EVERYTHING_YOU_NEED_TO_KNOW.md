# Everything You Need to Know

## System data flow

Gazebo publishes simulated RGB, depth, LiDAR, IMU, odometry, and clock data.
ROS-Gazebo bridges convert those messages to ROS 2. PX4 supplies flight
commands/state for the UAV. Nav2 consumes the Panther occupancy map and LiDAR.
The UAV tracker consumes the camera image/depth and publishes velocity commands.

```
Gazebo sensors -> ros_gz_bridge -> ROS topics
PX4/QGroundControl -> UAV flight state
Panther LiDAR -> Nav2 costmaps -> /cmd_vel -> Panther
UAV RGB-D -> ArUco detector -> visual_tag_follower -> UAV velocity
```

## Main launch files


### Terminal rule

Before every ROS command in a new terminal, run:

```bash
cd ~/husarion_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

The complete launch files start their own world and dependencies. Diagnostic
commands and standalone navigation launches do not; their prerequisites are
listed below.

* `src/uav_groundtruth_mapping/launch/complete_visual_follow_harvest.launch.py`:
  complete current integration launch.
* `src/uav_gps_mapping/launch/complete_gps_mapping.launch.py`: PX4-pose map,
  filtering, traversability mask, and optional Panther navigation.
* `src/uav_groundtruth_mapping/launch/complete_groundtruth_mapping.launch.py`:
  exact Gazebo-pose reference mapping.
* `src/uav_rtabmap_mapping/launch/complete_rtabmap_mapping.launch.py`:
  isolated RGB-D RTAB-Map experiment.
* `src/uav_groundtruth_mapping/launch/panther_nav_from_row_mask.launch.py`:
  Nav2 from the row-corridor mask.
* `src/uav_gps_mapping/launch/panther_nav_from_latest.launch.py`: Nav2 from
  the newest GPS-derived map. Requires Gazebo/Panther and the map products to
  already be running/available; it does not start the world.

`complete_visual_follow_harvest.launch.py`, `complete_gps_mapping.launch.py`,
`complete_groundtruth_mapping.launch.py`, and
`complete_rtabmap_mapping.launch.py` each start their own world. Do not run a
second Gazebo or PX4 launch alongside them. `panther_nav_from_*` launches are
the exceptions: they attach to an already-running simulator.

## Mapping algorithms and files

### GPS/PX4 mapper

`uav_gps_mapping/uav_gps_mapping/gps_pointcloud_mapper.py` receives each
LiDAR scan, transforms it with PX4 local position and attitude, and inserts
points into a voxel map. Optional bounded ICP estimates a small translation
correction by minimizing point-to-nearest-neighbor error. Corrections are
limited and rejected when they make the fit worse, so PX4 remains the main
pose source. The mapper writes PCD files on mission completion or through:

```bash
ros2 service call /gps_pointcloud_mapper/save std_srvs/srv/Trigger '{}'
```

`pcd_to_occupancy.py` projects the 3D cloud into a top-down grid. It removes
terrain below the configured height band, marks vegetation/obstacles, then
inflates obstacles by the Panther radius and safety margin. The result is a
Nav2 occupancy image and YAML metadata.

### Ground-truth mapper

`uav_groundtruth_mapping/uav_groundtruth_mapping/groundtruth_pointcloud_mapper.py`
uses the Gazebo model pose rather than estimating pose. This makes it the
reference answer for simulator geometry. `row_corridor_mask.py` detects the
repeated row direction/spacing, separates green corridors from red inflated
vegetation, and writes centerlines for the serpentine mission.

### RTAB-Map

The RTAB launch/configuration is in `src/uav_rtabmap_mapping`. RGB and depth
are registered with visual odometry and loop closure. Its database is a `.db`
file; open it with RTAB-Map or export its point cloud rather than executing it
from a shell.

## Panther navigation

`serpentine_mission_manager.py` loads
`latest_panther_row_corridors_centerlines.csv`, validates goals against the
Nav2 map, and sends one row at a time. It alternates direction at each
headland. Nav2's MPPI controller (`config/gps_denied_nav2.yaml`) samples
candidate velocity sequences and scores collision cost, path alignment,
goal distance, and forward preference. `native_panther_drive.py` supplies the
simulator's direct drive interface. The yellow safety walls in
`sdf/orchard_safety_walls.sdf` are collision boundaries only; they are not
navigation waypoints and are not used as the row path.

## UAV marker search and tracking

The implementation is in
`src/uav_ugv_control/uav_ugv_control/visual_tag_follower.py`.

1. The Panther starts first. A route-prior point list is received from the
   mission manager. After the head-start delay and operator authorization, the
   UAV follows a conservative intercept search route.
2. The RGB image is checked for the configured ArUco marker (default ID 0).
   The depth image supplies marker range. A quiet-zone/verification step avoids
   accepting noise as a detection.
3. Once visible, pixel error controls horizontal motion and depth error controls
   altitude. For image width `W`, height `H`, focal lengths `fx, fy`, marker
   pixel `(u,v)`, and depth `Z`:

   ```text
   eu = u - W/2,        ev = v - H/2
   vx = -Kx * ev * Z/fy
   vy = -Ky * eu * Z/fx
   vz = Kv * (Z - desired_depth)
   ```

   Commands are dead-band limited and clipped to configured speed/acceleration
   limits. The camera is mounted downward, so image axes are rotated into the
   simulator's horizontal axes.
4. Short marker loss performs local reacquisition around the last sighting;
   longer loss returns to route interception. The UAV does not need GPS for
   visual following.

The prior experiment added `marker_position_relative_to_uav()` and
`filtered_target_velocity()` to estimate Panther speed from consecutive camera
relative positions, with UAV ego-motion compensation. That experiment is
backed up but feed-forward is currently disabled (`velocity_feedforward_gain=0`)
to restore the pre-speed-estimator behavior. The code can be re-enabled in a
future version without changing this baseline.

`uav_camera_viewer.py` displays the downward UAV camera and provides the
**START LOCATING** button. It calls `/uav/tag_follower/start_locating`; it does
not detect the marker itself.

The current `minimum_detection_confidence` is `0.35` (previously `0.85`) to
accept blurred or partially occluded markers. Lowering this threshold can
increase false positives; inspect `/uav/tag_follower/detection` and verify the
marker ID and confidence during tests.

## RL landing integration

`uav_ugv_control/rl_landing_policy.py` adapts the upstream
`rl_multi_rotor_landing` method to ROS 2 without importing its ROS Noetic
workspaces. `RelativeState` stores marker position and estimated Panther
velocity in the UAV camera/body frame. `Discretizer` clips each value to a
bounded multi-resolution grid. `QLandingPolicy` maps a discretized state to a
Q-table action; an absent or invalid table returns hover. `cascaded_velocity`
is the bounded deterministic fallback.

`visual_tag_follower.py` uses these primitives only when
`enable_rl_landing:=true`. It enters landing approach below
`landing_start_depth`, requires a valid marker/depth observation, and holds
position at `landing_touchdown_depth`. The status topic reports
`landing_state` as `DISABLED`, `RL_APPROACH`, `PI_FALLBACK`, or
`TOUCHDOWN_HOLD`. The launch parameters are:

```text
enable_rl_landing       false
landing_start_depth     3.0
landing_touchdown_depth 0.35
rl_policy_path          ""
```

The current implementation is safe inference/fallback infrastructure, not a
trained policy. Training is required only to replace the PI fallback with a
learned landing policy. Training must be done in randomized simulation first;
do not enable an unvalidated table on hardware. Backups for this integration
are in `/home/robocare/backups/version_20260902_rl_landing_integration/`.

## Frames and timestamps

The active visual controller uses the UAV camera-relative measurement for
marker centering. The initial route search and recovery use a dead-reckoned
map/world estimate only to choose where to look next. Mapping clouds are
normally expressed in `world`; Panther Nav2 uses `map` and `panther/base_link`.
All nodes use `/clock` and `use_sim_time`; stale messages after a simulator
restart can cause TF “timestamp earlier than cache” warnings. Stop the old
launch completely and start one fresh copy if that occurs.

If only windows appear and the UAV/Panther are missing, the world server and
delayed spawn actions are out of sync (often because an older world is still
running). Stop all launch terminals, verify no old `gz sim` process remains,
and rerun `complete_visual_follow_harvest.launch.py` once. Allow its 20-second
startup sequence to finish before judging the scene.

## Useful diagnostics

```bash
ros2 topic list | rg 'uav|panther|cloud|image|depth|cmd_vel'
ros2 topic hz /uav/tag_camera/image_raw
ros2 topic hz /uav/tag_camera/depth_image
ros2 topic echo /uav/tag_follower/status
ros2 topic echo /tf --once
ros2 node list
```

PCD viewing:

```bash
pcl_viewer "$(ls -t ~/husarion_ws/maps/uav_gps_map_*.pcd | head -n1)"
```

A `.pcd` path alone gives “Permission denied” because it is not a program;
pass it as an argument to `pcl_viewer`.

## QGroundControl

QGroundControl is used to upload/start UAV missions and provide PX4 telemetry.
You do not need to add camera “Take photo” mission items for the current
tracker: the camera stream is continuous and ROS performs detection. Invalid
camera mission items can cause QGroundControl’s “Param 5 invalid” error.
