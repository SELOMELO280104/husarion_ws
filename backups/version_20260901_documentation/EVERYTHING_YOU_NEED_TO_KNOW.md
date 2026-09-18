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
  the newest GPS-derived map.

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

## Frames and timestamps

The active visual controller uses the UAV camera-relative measurement for
marker centering. The initial route search and recovery use a dead-reckoned
map/world estimate only to choose where to look next. Mapping clouds are
normally expressed in `world`; Panther Nav2 uses `map` and `panther/base_link`.
All nodes use `/clock` and `use_sim_time`; stale messages after a simulator
restart can cause TF “timestamp earlier than cache” warnings. Stop the old
launch completely and start one fresh copy if that occurs.

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
