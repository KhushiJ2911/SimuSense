cmake_minimum_required(VERSION 3.0.2)
project(vision_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

# Specify Python Scripts as Executables
install(PROGRAMS
  src/ai_perception_node.py
  src/p_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Launch and Config Files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
