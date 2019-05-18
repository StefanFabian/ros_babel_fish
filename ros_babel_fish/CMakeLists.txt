cmake_minimum_required(VERSION 2.8.3)
project(ros_babel_fish)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-Wall -Wextra)

#set(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp)

find_package(PythonLibs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_babel_fish
  CATKIN_DEPENDS roscpp
  DEPENDS PYTHON
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

set(SOURCES
  src/message_types/array_message.cpp
  src/message_types/compound_message.cpp
  src/message_types/value_message.cpp
  src/babel_fish.cpp
  src/babel_fish_message.cpp
  src/message.cpp
  src/message_creation.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${BOOST_LIBRARIES})

## Declare examples as C++ executables
add_executable(any_publisher examples/any_publisher.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(any_publisher ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(any_subscriber examples/any_subscriber.cpp)
target_link_libraries(any_subscriber ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(message_info examples/message_info.cpp)
target_link_libraries(message_info ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(troll_node examples/troll_node.cpp)
target_link_libraries(troll_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(service_server examples/service_server.cpp)
target_link_libraries(service_server ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(service_client examples/service_client.cpp)
target_link_libraries(service_client ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
