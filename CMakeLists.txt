cmake_minimum_required(VERSION 3.0.2)
project(apriltag_box_detection)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  cv_bridge
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

add_message_files(
  FILES
  BoxPose.msg
  BoxPoseArray.msg
  BoxTwist.msg
  BoxTwistArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)

catkin_install_python(PROGRAMS
  scripts/box_image_overlay.py
  scripts/box_pose_estimator.py
  scripts/box_config_viewer.py
  scripts/robot_config_viewer.py
  scripts/robot_relative_box_pose_estimator.py
  scripts/generate_tag_meshes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY assets config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
