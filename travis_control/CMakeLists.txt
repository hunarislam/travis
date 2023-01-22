cmake_minimum_required(VERSION 3.0.2)
project(travis_control)

find_package(catkin REQUIRED COMPONENTS
  joy
  rospy
  std_msgs
  geometry_msgs
)

# catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs std_msgs rospy
)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS nodes/xbox_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)