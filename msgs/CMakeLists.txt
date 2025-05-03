cmake_minimum_required(VERSION 3.0.2)
project(msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

add_action_files(
  FILES
  Raise.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)