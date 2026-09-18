cmake_minimum_required(VERSION 3.8)
project(uav_rtabmap_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/resolve_orchard_world.py
  scripts/gazebo_pose_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
