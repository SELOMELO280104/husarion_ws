# UAV RTAB-Map experiment

This package is independent of `uav_gps_mapping` and
`uav_terrain_mapping`. It does not replace or load either map.

Start the isolated Orchard + PX4 + RTAB-Map experiment with:

```bash
mkdir -p ~/husarion_ws/maps/rtabmap
source /opt/ros/jazzy/setup.bash
source ~/husarion_ws/install/setup.bash
ROS_DOMAIN_ID=0 ros2 launch uav_rtabmap_mapping complete_rtabmap_mapping.launch.py
```

Before flying, verify:

```bash
ROS_DOMAIN_ID=0 ros2 topic hz /uav/rtabmap/rgb/image_raw
ROS_DOMAIN_ID=0 ros2 topic hz /uav/rtabmap/depth/image_raw
ROS_DOMAIN_ID=0 ros2 topic echo /uav/rtabmap/rgb/camera_info --once
```

RTAB-Map writes its database to:

```text
~/husarion_ws/maps/rtabmap/uav_orchard.db
```

The colored reconstruction uses RGB-D data. Gazebo camera-link ground-truth
odometry is supplied on `/uav/rtabmap/odom` so repetitive orchard imagery
cannot reset the trajectory.

The default camera frame is a hypothesis based on Gazebo's scoped sensor
name. Confirm the actual `CameraInfo.header.frame_id` on the first run and
pass it with `camera_frame:=...` if it differs.
