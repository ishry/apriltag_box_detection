cmake_minimum_required(VERSION 3.0.2)
project(apriltag_box_detection)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  rospy
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/box_pose_estimator.py
  scripts/box_config_viewer.py
  scripts/generate_tag_meshes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY assets config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
