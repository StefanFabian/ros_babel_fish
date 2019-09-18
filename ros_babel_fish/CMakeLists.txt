cmake_minimum_required(VERSION 3.10)
project(ros_babel_fish)

# If the value doesn't fit, an exception is thrown in any case because that could result in unexpected behavior and can not be ignored lightly
option(WARN_ON_INCOMPATIBLE_TYPE "If ON a warning is printed if a value message is set or accessed with a type that does not allow casting without loss of information" ON)
option(ENABLE_EMBEDDED_PYTHON "Enables support for embedded python description provider" OFF)

if (WARN_ON_INCOMPATIBLE_TYPE)
  add_definitions(-DRBF_WARN_ON_INCOMPATIBLE_TYPE)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-Wall -Wextra)

#set(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslib)
find_package(OpenSSL REQUIRED)

set(DEPENDS )

if (ENABLE_EMBEDDED_PYTHON)
  find_package(PythonInterp)
  find_package(PythonLibs 2.7 EXACT REQUIRED)

  message(STATUS "Using Python ${PYTHON_VERSION_STRING}")

  add_definitions(-DRBF_ENABLE_EMBEDDED_PYTHON)
endif()

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
  CATKIN_DEPENDS roscpp roslib
  DEPENDS ${DEPENDS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenSSL_INCLUDE_DIRS}
)

set(LIBRARIES
  ${catkin_LIBRARIES}
  OpenSSL::SSL
  stdc++fs
)

set(SOURCES
  src/generation/providers/integrated_description_provider.cpp
  src/generation/description_provider.cpp
  src/generation/message_creation.cpp
  src/messages/array_message.cpp
  src/messages/compound_message.cpp
  src/messages/value_message.cpp
  src/babel_fish.cpp
  src/babel_fish_message.cpp
  src/message.cpp
  src/message_extractor.cpp
)

if (ENABLE_EMBEDDED_PYTHON)
  include_directories(${PYTHON_INCLUDE_DIRS})
  set(LIBRARIES ${LIBRARIES} ${PYTHON_LIBRARIES})
  set(SOURCES ${SOURCES} src/generation/providers/embedded_python_description_provider.cpp)
endif()


## Declare a C++ library
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

## Declare examples as C++ executables
add_executable(any_publisher examples/any_publisher.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(any_publisher ${PROJECT_NAME} ${LIBRARIES})

add_executable(any_subscriber examples/any_subscriber.cpp)
target_link_libraries(any_subscriber ${PROJECT_NAME} ${LIBRARIES})

add_executable(message_info examples/message_info.cpp)
target_link_libraries(message_info ${PROJECT_NAME} ${LIBRARIES})

add_executable(troll_node examples/troll_node.cpp)
target_link_libraries(troll_node ${PROJECT_NAME} ${LIBRARIES})

add_executable(service_server examples/service_server.cpp)
target_link_libraries(service_server ${PROJECT_NAME} ${LIBRARIES})

add_executable(service_client examples/service_client.cpp)
target_link_libraries(service_client ${PROJECT_NAME} ${LIBRARIES})

#find_package(actionlib)

#if(${actionlib_FOUND})
#  add_executable(action_client examples/action_client.cpp)
#  target_link_libraries(action_client ${PROJECT_NAME} ${catkin_LIBRARIES} ${actionlib_LIBRARIES})
#  add_executable(simple_action_client examples/simple_action_client.cpp)
#  target_link_libraries(simple_action_client ${PROJECT_NAME} ${catkin_LIBRARIES} ${actionlib_LIBRARIES})
  #add_executable(action_server examples/action_server.cpp)
  #target_link_libraries(action_server ${PROJECT_NAME} ${catkin_LIBRARIES} ${actionlib_LIBRARIES})
#endif()


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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  any_publisher
  any_subscriber
  message_info
  troll_node
  service_server
  service_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_babel_fish_test_msgs REQUIRED)
  include_directories(${ros_babel_fish_test_msgs_INCLUDE_DIRS})

  add_rostest_gtest(test_message test/test_message.test test/message.cpp)
  target_link_libraries(test_message ${PROJECT_NAME})

  add_rostest_gtest(test_message_decoding test/test_message_decoding.test test/message_decoding.cpp)
  target_link_libraries(test_message_decoding ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})

  add_rostest_gtest(test_message_encoding test/test_message_encoding.test test/message_encoding.cpp)
  target_link_libraries(test_message_encoding ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})

  add_rostest_gtest(test_message_extractor test/test_message_extractor.test test/message_extractor.cpp)
  target_link_libraries(test_message_extractor ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})

  add_rostest_gtest(test_message_lookup test/test_message_lookup.test test/message_lookup.cpp)
  target_link_libraries(test_message_lookup ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})

  add_rostest_gtest(test_service_lookup test/test_service_lookup.test test/service_lookup.cpp)
  target_link_libraries(test_service_lookup ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})

  add_rostest_gtest(test_service_client test/test_service_client.test test/service_client.cpp)
  target_link_libraries(test_service_client ${PROJECT_NAME})

  add_executable(test_service_client_services test/service_client_test_services.cpp)
  target_link_libraries(test_service_client_services ${PROJECT_NAME})
endif()

# to run: catkin build -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args ros_babel_fish_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information and code to get the coverage
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/examples*" "*/${PROJECT_NAME}/benchmarks*")
  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif()
