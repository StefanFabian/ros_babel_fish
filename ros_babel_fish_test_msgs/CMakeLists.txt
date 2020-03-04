cmake_minimum_required(VERSION 2.8.3)
project(ros_babel_fish_test_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation std_msgs geometry_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TestArray.msg
  TestMessage.msg
  TestSubArray.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  SimpleTest.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs  # Or other packages containing msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs geometry_msgs
)
