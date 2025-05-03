cmake_minimum_required(VERSION 3.0.2)
project(actions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  common
  kinematics
)

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs common kinematics
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/kick.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  kick/kick.py
  kick/chute_raw_angle.py
  getup/levante.py
  getup/getup_imu.py
  getup/levante_frente_raw_angle.py
  getup/levante_tras_raw_angle.py
  getup/levante_short_frente.py
  getup/levante_short_tras.py
  getup/levantamento_imu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)