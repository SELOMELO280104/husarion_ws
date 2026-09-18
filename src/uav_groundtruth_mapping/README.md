# Gazebo ground-truth UAV mapping

This package keeps PX4 and QGroundControl in control of the UAV but registers
the RGB LiDAR scans with the exact Gazebo model pose. It is intended as a
simulation reference map and not as an estimate of real-world GPS accuracy.

Run:

```bash
cd ~/husarion_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ros2 launch uav_groundtruth_mapping \
  complete_groundtruth_mapping.launch.py
```

Mapping starts when PX4 enters `MISSION` / `AUTO.MISSION` and saves when the
mission ends. Each mission produces:

- `uav_groundtruth_map_<timestamp>.pcd`
- `uav_groundtruth_map_<timestamp>_filtered.pcd`
- `latest_panther_traversability.pcd`
- `latest_panther_traversability_nav.yaml`
- `latest_panther_row_corridors.pcd`
- `latest_panther_row_corridors.ppm`
- `latest_panther_row_corridors_nav.pgm/.yaml`
- `latest_panther_row_corridors_centerlines.csv`

The filtered PCD is the recommended viewer and navigation input.

The row-corridor products estimate ground height, detect the direction and
spacing of the repeated vegetation rows, keep obstacles within the Panther
body-height collision band, and inflate them by the 0.65 m robot radius plus
0.10 m safety margin. In the PCD / PPM overlay:

- green is reachable space between rows and in the turning headlands;
- red is footprint-inflated vegetation;
- blue is a detected corridor centerline;
- yellow is the selected Panther start.

Regenerate the mask from the newest ground-truth PCD:

```bash
ros2 launch uav_groundtruth_mapping generate_row_corridor_mask.launch.py
```

Inspect the 3D scan and corridor mask together:

```bash
LATEST_GT=$(find ~/husarion_ws/maps -maxdepth 1 -type f \
  -name 'uav_groundtruth_map_*_filtered.pcd' \
  -printf '%T@ %p\n' | sort -nr | head -n1 | cut -d' ' -f2-)
pcl_viewer "$LATEST_GT" \
  ~/husarion_ws/maps/latest_panther_row_corridors.pcd
```

With the Panther simulation already running, start Nav2 using only the
row-corridor map:

```bash
ros2 launch uav_groundtruth_mapping panther_nav_from_row_mask.launch.py
```

The default PX4 instance is spawned as `x500_depth_gps_0`, whose exact Gazebo
pose is bridged from `/model/x500_depth_gps_0/pose`. If the model instance name
is changed, pass the matching `gz_pose_topic` launch argument.

## GPS-denied Panther-first visual search

Run the Panther harvest route, delayed UAV search, marker acquisition and
visual following in one launch:

```bash
cd ~/husarion_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ros2 launch uav_groundtruth_mapping \
  complete_visual_follow_harvest.launch.py
```

The Panther starts as soon as Nav2 and its safety inputs are ready. Press
**START LOCATING** in the dedicated UAV camera window to authorize the UAV.
The UAV remains stationary until both that button has been pressed and
`uav_search_delay` seconds have elapsed since the Panther reported `RUNNING`.
It then searches the known route around a conservative progress prediction,
verifies ArUco ID 0 in the downward RGB-D image and changes to visual
following. A short marker loss triggers an expanding search around the last
sighting; a longer loss returns to route interception. No live Panther pose or
GPS measurement is sent to the UAV.

After marker acquisition, tracking is UAV-camera-relative. Consecutive RGB-D
marker positions estimate Panther velocity by subtracting UAV ego-motion, and
a filtered fraction of that estimate is used as velocity feed-forward. The
status topic reports `marker_relative_xy_m`,
`estimated_panther_velocity_mps`, and `estimated_panther_speed_mps`.

Useful tuning example:

```bash
ROS_DOMAIN_ID=0 ros2 launch uav_groundtruth_mapping \
  complete_visual_follow_harvest.launch.py \
  uav_search_delay:=30.0 \
  ugv_nominal_speed:=0.32 \
  route_sweep_distance:=4.0
```

Monitor the state machine with:

```bash
ROS_DOMAIN_ID=0 ros2 topic echo /uav/tag_follower/status
```
