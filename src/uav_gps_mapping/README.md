# PX4 GPS/INS 3D mapping

PX4's EKF provides `LOCAL_POSITION_NED` and `ATTITUDE_QUATERNION` over MAVLink.
Each 32-channel Gazebo LiDAR scan is first transformed into the ROS `world`
ENU frame with that pose, then refined by bounded scan-to-map ICP before it is
voxel-accumulated. ICP is translation-only by default and can correct at most
0.20 m, so PX4 remains the position and attitude reference. Flat or ambiguous
scans and corrections that do not materially improve the fit are rejected
automatically.

```bash
cd ~/husarion_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ros2 launch uav_gps_mapping complete_gps_mapping.launch.py
```

The launch enables ICP by default. For an A/B comparison, disable it with
`enable_icp:=false`. Status lines report accepted/attempted corrections and
the most recent before/after RMSE.

Upload and start the flight plan in QGroundControl. Live products are:

- `/gps_pointcloud/registered_scan`
- `/gps_pointcloud/map`
- `/gps_pointcloud/path`

Each mission writes two maps:

- `uav_gps_map_<timestamp>.pcd`: the unchanged voxelized map.
- `uav_gps_map_<timestamp>_filtered.pcd`: the recommended viewer/navigation
  map after statistical outlier removal.

A manual save is also available while points exist:

```bash
ros2 service call /gps_pointcloud_mapper/save std_srvs/srv/Trigger
```

The default 3D voxel is 10 cm. Use `voxel_size:=0.05` for a denser scan if
the additional memory and file size are acceptable.

Each save also generates a Panther-footprint traversability overlay:

- `latest_panther_traversability.pcd`: green reachable space, red inflated
  obstacles, and a blue start marker.
- `latest_panther_traversability.ppm`: top-down color preview.
- `latest_panther_traversability_nav.pgm/.yaml`: the same mask in a
  Nav2-compatible format.

The mask uses a 0.65 m Panther radius plus 0.10 m safety clearance. Low terrain
below 0.35 m and foliage above the Panther collision band are not treated as
tree obstacles.

## Panther navigation from the latest GPS map

`complete_gps_mapping.launch.py` now converts the newest nonempty
`uav_gps_map_*.pcd` into the Panther traversability mask at startup and starts
Nav2 with `latest_panther_traversability_nav.yaml`. In RViz, use **Nav2 Goal**
to choose a point in the green reachable region.
Nav2 publishes through Panther's low-priority autonomous velocity input, so
manual control remains higher priority.

To regenerate or inspect the 2D map without launching the simulation:

```bash
ros2 run uav_gps_mapping pcd_to_occupancy
ros2 launch uav_gps_mapping panther_nav_from_latest.launch.py
```

To inspect the colored 3D scan and mask together:

```bash
pcl_viewer \
  "$(ls -t ~/husarion_ws/maps/uav_gps_map_*.pcd | head -n 1)" \
  ~/husarion_ws/maps/latest_panther_traversability.pcd
```
