# PX4 + Gazebo + ROS 2 Orchard Simulation

This workspace contains the current orchard simulation for the PX4 UAV and
Husarion Panther UGV. Commands below assume ROS 2 Jazzy and a bash shell.

## 1. Start from a clean terminal

```bash
cd ~/husarion_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

## 2. Complete simulation (recommended)

This is the one-command startup for Gazebo, PX4, the Panther, Nav2, RViz,
the UAV camera viewer, QGroundControl connectivity, and the mission manager:

```bash
ros2 launch uav_groundtruth_mapping complete_visual_follow_harvest.launch.py
```

The Panther starts its GPS-denied serpentine route after Nav2 is ready. The
UAV remains on the ground for the configured head start (30 seconds by
default). Press **START LOCATING** in the UAV camera window to authorize the
search. The UAV then searches the Panther route and follows ArUco marker ID 0
when it is visible. Locating is not started automatically.

Stop everything with `Ctrl-C` in the launch terminal. Do not start a second
copy of the launch while the first one is running.

## 3. Important controls and status

```bash
ros2 topic echo /uav/tag_follower/status
ros2 service call /uav/tag_follower/start_locating std_srvs/srv/Trigger '{}'
ros2 service call /panther/harvest/start std_srvs/srv/Trigger '{}'
```

The service call is equivalent to pressing **START LOCATING**. The status
machine normally progresses through `WAITING_FOR_OPERATOR`, route search,
`VISUAL_FOLLOW_VELOCITY`, and completion. `gps_used: false` is expected.

## 4. Mapping choices

### PX4/GPS pose map (current simple 3D map)

```bash
ros2 launch uav_gps_mapping complete_gps_mapping.launch.py
```

Upload/start the mission in QGroundControl. The mapper saves files under
`~/husarion_ws/maps/`:

* `uav_gps_map_<timestamp>.pcd` — voxelized point cloud.
* `uav_gps_map_<timestamp>_filtered.pcd` — filtered recommended cloud.
* `latest_panther_traversability.pcd/.ppm` and Nav2 `.pgm/.yaml` — obstacle
  mask and navigable-space products.

Open the newest cloud (a PCD is data, not an executable):

```bash
pcl_viewer "$(find ~/husarion_ws/maps -maxdepth 1 -name 'uav_gps_map_*.pcd' -printf '%T@ %p\n' | sort -nr | head -n1 | cut -d' ' -f2-)"
```

Use `voxel_size:=0.10` for the normal map, or `voxel_size:=0.05` for a denser,
larger map. `enable_icp:=true` enables bounded scan refinement.

### Gazebo ground-truth reference map

```bash
ros2 launch uav_groundtruth_mapping complete_groundtruth_mapping.launch.py
```

This is the most geometrically accurate simulator reference because it uses
the exact Gazebo pose. It produces `uav_groundtruth_map_*.pcd` and the same
Panther traversability/row-corridor products. It is for simulation evaluation,
not a real-world localization method.

### RTAB-Map RGB-D experiment

```bash
mkdir -p ~/husarion_ws/maps/rtabmap
ros2 launch uav_rtabmap_mapping complete_rtabmap_mapping.launch.py
```

RTAB-Map uses RGB-D registration and saves `~/husarion_ws/maps/rtabmap/uav_orchard.db`.
It is independent of the GPS and ground-truth mappers.

## 5. Panther navigation from a map

The complete GPS mapping launch can start navigation from the latest GPS map.
For an already-running simulator:

```bash
ros2 launch uav_gps_mapping panther_nav_from_latest.launch.py
```

For the ground-truth row mask:

```bash
ros2 launch uav_groundtruth_mapping panther_nav_from_row_mask.launch.py
```

Green regions in the mask are reachable space; red regions are inflated
obstacles. Nav2 publishes autonomous velocity while manual teleoperation has
higher priority.

## 6. Version and backup rule

Before every source or configuration change, copy the affected files into a
new directory under `~/husarion_ws/backups/version_<YYYYMMDD>_<description>/`
and add a `CHANGELOG.txt` describing the change. The current speed/origin
experiment is preserved in:

`backups/version_20260901_uav_speed_origin/`

Always rebuild after source changes:

```bash
colcon build --packages-select uav_ugv_control uav_groundtruth_mapping uav_gps_mapping --symlink-install
source install/setup.bash
```

See [EVERYTHING_YOU_NEED_TO_KNOW.md](EVERYTHING_YOU_NEED_TO_KNOW.md) for the
file-by-file architecture and algorithm mathematics.
