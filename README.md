# PX4 + Gazebo + ROS 2 Orchard Simulation

This workspace contains the current orchard simulation for the PX4 UAV and
Husarion Panther UGV. Commands below assume ROS 2 Jazzy and a bash shell.


Editor`s note!!!!: some information on the read me and everything you need to know file can be wrong. I did not have time to check and fix all of them. This repository is currently acting as a backup file so people can at least use the methodologies used in this project.



# Dependencies & Installation

This repository contains the custom control and mapping packages for the UAV-UGV orchard simulation. To run this project on a local machine, you must install the required external dependencies and third-party packages that are excluded from version control.

## 1. Prerequisites
* **ROS 2 Jazzy:** [Official Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
* **Gazebo Harmonic:** Installed alongside ROS 2 Jazzy.
* **PX4 Autopilot:** Must be installed in your home directory.
  ```bash
  cd ~
  git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) --recursive
  bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

## 2. Workspace Setup

Clone this repository to create your workspace base:
Bash

mkdir -p ~/husarion_ws
cd ~/husarion_ws
git clone [https://github.com/SELOMELO280104/husarion_ws.git](https://github.com/SELOMELO280104/husarion_ws.git) .

## 3. Cloning Third-Party Dependencies

Navigate to the src folder and clone the essential PX4, Husarion, and BehaviorTree packages:
Bash

cd ~/husarion_ws/src

### PX4 Messages
git clone [https://github.com/PX4/px4_msgs.git](https://github.com/PX4/px4_msgs.git)

### BehaviorTree (Required for Navigation2)
git clone [https://github.com/BehaviorTree/BehaviorTree.CPP.git](https://github.com/BehaviorTree/BehaviorTree.CPP.git)
git clone [https://github.com/BehaviorTree/BehaviorTree.ROS2.git](https://github.com/BehaviorTree/BehaviorTree.ROS2.git)

### Husarion Panther & Gazebo Simulation Packages
git clone [https://github.com/husarion/husarion_components_description.git](https://github.com/husarion/husarion_components_description.git)
git clone [https://github.com/husarion/husarion_controllers.git](https://github.com/husarion/husarion_controllers.git)
git clone [https://github.com/husarion/husarion_gz_worlds.git](https://github.com/husarion/husarion_gz_worlds.git)
git clone [https://github.com/husarion/husarion_ugv_ros.git](https://github.com/husarion/husarion_ugv_ros.git)
git clone [https://github.com/husarion/joy2twist.git](https://github.com/husarion/joy2twist.git)

## 4. Install Dependencies & Build

Use rosdep to install any missing underlying dependencies before building the workspace:
Bash

cd ~/husarion_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# 1. Start from a clean terminal

Every command that starts a ROS node must be run in a terminal where the
workspace has been sourced. These four lines are required in each new
terminal (unless it is only running a non-ROS viewer such as `pcl_viewer`):

```bash
cd ~/husarion_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

The launch files set these values for their child processes too, but sourcing
them in the calling terminal is still required for `ros2 topic`, `ros2
service`, and `ros2 node` commands. If a launch is already running, open a
second terminal and repeat the four setup lines before running a diagnostic or
service command.

# 2. Complete simulation (recommended)

This is the one-command startup for Gazebo, PX4, the Panther, Nav2, RViz,
the UAV camera viewer, QGroundControl connectivity, and the mission manager:

```bash
ros2 launch uav_groundtruth_mapping complete_visual_follow_harvest.launch.py

```
we also added some more changes for example 0 delay start and tracking/landing nodes they should be running before the tracking process starts

open a new terminal and paste the command on the bottom lines this will be your terminal 1

```bash
cd ~/husarion_ws
source install/setup.bash
ros2 launch uav_groundtruth_mapping complete_visual_follow_harvest.launch.py enable_rl_landing:=true landing_touchdown_depth:=0.35 uav_search_delay:=0

```
tracking node on 2nd terminal

```bash
cd ~/husarion_ws
source install/setup.bash
python3 src/uav_ugv_control/uav_ugv_control/hybrid_tracker.py

```



No other terminal is needed: this launch starts the orchard world and both
vehicles itself. Do not start Gazebo, PX4, or a separate Panther launch first.
Wait about 20 seconds for the delayed spawners. The expected windows are
Gazebo, RViz, QGroundControl (if installed/configured), and the UAV camera
viewer. The Panther is spawned at `(-6,-8)` and the UAV at `(-6,-9,0.8)`;
use Gazebo's **Follow**/**View** controls or zoom out if they are outside the
initial camera view.

If Gazebo/RViz/camera open but neither vehicle appears, an older Gazebo launch
is usually still running or the launch was interrupted during its delayed
spawn sequence. Close the old Gazebo/RViz windows, press `Ctrl-C` in every
old ROS launch terminal, then run only the command above from a freshly
sourced terminal. Check that the launch log contains `spawn` processes and
that `ros2 node list` includes `native_panther_drive` and
`uav_visual_tag_follower`.

The Panther starts its GPS-denied serpentine route after Nav2 is ready. The
UAV remains on the ground for the configured head start (30 seconds by
default). Press **START LOCATING** in the UAV camera window to authorize the
search. The UAV then searches the Panther route and follows ArUco marker ID 0
when it is visible. Locating is not started automatically.

Stop everything with `Ctrl-C` in the launch terminal. Do not start a second
copy of the launch while the first one is running.

### Optional RL landing integration

The current tracker includes a safety-gated adapter based on the relative-state
Q-learning and cascaded-control methods from the upstream landing project. It
is disabled by default. Enable the landing state machine (using the safe PI
fallback until a policy is trained) with:

```bash
ros2 launch uav_groundtruth_mapping complete_visual_follow_harvest.launch.py \
  enable_rl_landing:=true
```

Parameters are `landing_start_depth` (default 3.0 m),
`landing_touchdown_depth` (default 0.35 m), and `rl_policy_path` (optional JSON
Q-table). No training is required for normal tracking; training is required
before using a learned Q-table for autonomous landing.

## 3. Important controls and status

```bash
ros2 topic echo /uav/tag_follower/status
ros2 service call /uav/tag_follower/start_locating std_srvs/srv/Trigger '{}'
ros2 service call /panther/harvest/start std_srvs/srv/Trigger '{}'
```

The service call is equivalent to pressing **START LOCATING**. The status
machine normally progresses through `WAITING_FOR_OPERATOR`, route search,
`VISUAL_FOLLOW_VELOCITY`, and completion. `gps_used: false` is expected.

Marker acceptance currently uses a minimum confidence of `0.35` to tolerate
blur and partial occlusion. Monitor detections while testing:

```bash
ros2 topic echo /uav/tag_follower/detection
```

Lower confidence can increase false positives; verify the marker ID and
confidence before relying on autonomous motion.

## 4. Mapping choices

### PX4/GPS pose map (current simple 3D map)

Prerequisite: no separate world is required. This launch starts Gazebo, PX4,
the UAV, bridges, RViz, and the mapper. Run it alone from a newly sourced
terminal:

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

Prerequisite: no separate world is required; this launch starts its own
Gazebo world and UAV.

```bash
ros2 launch uav_groundtruth_mapping complete_groundtruth_mapping.launch.py
```

This is the most geometrically accurate simulator reference because it uses
the exact Gazebo pose. It produces `uav_groundtruth_map_*.pcd` and the same
Panther traversability/row-corridor products. It is for simulation evaluation,
not a real-world localization method.

### RTAB-Map RGB-D experiment

Prerequisite: no separate world is required; the complete RTAB-Map launch
starts the isolated world and camera pipeline.

```bash
mkdir -p ~/husarion_ws/maps/rtabmap
ros2 launch uav_rtabmap_mapping complete_rtabmap_mapping.launch.py
```

RTAB-Map uses RGB-D registration and saves `~/husarion_ws/maps/rtabmap/uav_orchard.db`.
It is independent of the GPS and ground-truth mappers.

## 5. Panther navigation from a map

These navigation-only commands require a simulator/world and Panther sensor
topics to already be running. They do not spawn the UAV or start Gazebo.
Normally use them only after a mapping launch, or use the complete visual
launch in Section 2, which includes the required stack.

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
