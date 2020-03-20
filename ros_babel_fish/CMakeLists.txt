cmake_minimum_required(VERSION 3.0)
project(ros_babel_fish)

# If the value doesn't fit, an exception is thrown in any case because that could result in unexpected behavior and can not be ignored lightly
option(WARN_ON_INCOMPATIBLE_TYPE "If ON a warning is printed if a value message is set or accessed with a type that does not allow casting without loss of information" ON)

if (WARN_ON_INCOMPATIBLE_TYPE)
  add_definitions(-DRBF_WARN_ON_INCOMPATIBLE_TYPE)
endif ()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-Wall -Wextra)

#set(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib roscpp roslib std_msgs)
find_package(OpenSSL REQUIRED)

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
  CATKIN_DEPENDS roscpp roslib actionlib
  DEPENDS OPENSSL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
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


## Declare a C++ library
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

## Declare examples as C++ executables
add_executable(${PROJECT_NAME}_any_publisher examples/any_publisher.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_any_publisher ${PROJECT_NAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME}_any_publisher PROPERTIES OUTPUT_NAME any_publisher PREFIX "")

add_executable(${PROJECT_NAME}_any_subscriber examples/any_subscriber.cpp)
target_link_libraries(${PROJECT_NAME}_any_subscriber ${PROJECT_NAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME}_any_subscriber PROPERTIES OUTPUT_NAME any_subscriber PREFIX "")

add_executable(${PROJECT_NAME}_message_info examples/message_info.cpp)
target_link_libraries(${PROJECT_NAME}_message_info ${PROJECT_NAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME}_message_info PROPERTIES OUTPUT_NAME message_info PREFIX "")

add_executable(${PROJECT_NAME}_troll_node examples/troll_node.cpp)
target_link_libraries(${PROJECT_NAME}_troll_node ${PROJECT_NAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME}_troll_node PROPERTIES OUTPUT_NAME troll_node PREFIX "")

add_executable(${PROJECT_NAME}_service_server examples/service_server.cpp)
target_link_libraries(${PROJECT_NAME}_service_server ${PROJECT_NAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME}_service_server PROPERTIES OUTPUT_NAME service_server PREFIX "")

add_executable(${PROJECT_NAME}_service_client examples/service_client.cpp)
target_link_libraries(${PROJECT_NAME}_service_client ${PROJECT_NAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME}_service_client PROPERTIES OUTPUT_NAME service_client PREFIX "")

add_executable(${PROJECT_NAME}_action_client examples/action_client.cpp)
target_link_libraries(${PROJECT_NAME}_action_client ${PROJECT_NAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME}_action_client PROPERTIES OUTPUT_NAME action_client PREFIX "")


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  ${PROJECT_NAME}_action_client
  ${PROJECT_NAME}_any_publisher
  ${PROJECT_NAME}_any_subscriber
  ${PROJECT_NAME}_message_info
  ${PROJECT_NAME}_troll_node
  ${PROJECT_NAME}_service_server
  ${PROJECT_NAME}_service_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

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

  add_rostest_gtest(${PROJECT_NAME}_test_message test/test_message.test test/message.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_message PROPERTIES OUTPUT_NAME test_message PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_message_decoding test/test_message_decoding.test test/message_decoding.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message_decoding ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_test_message_decoding PROPERTIES OUTPUT_NAME test_message_decoding PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_message_encoding test/test_message_encoding.test test/message_encoding.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message_encoding ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_test_message_encoding PROPERTIES OUTPUT_NAME test_message_encoding PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_message_extractor test/test_message_extractor.test test/message_extractor.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message_extractor ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_test_message_extractor PROPERTIES OUTPUT_NAME test_message_extractor PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_message_lookup test/test_message_lookup.test test/message_lookup.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message_lookup ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_test_message_lookup PROPERTIES OUTPUT_NAME test_message_lookup PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_service_lookup test/test_service_lookup.test test/service_lookup.cpp)
  target_link_libraries(${PROJECT_NAME}_test_service_lookup ${PROJECT_NAME} ${ros_babel_fish_test_msgs_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_test_service_lookup PROPERTIES OUTPUT_NAME test_service_lookup PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_service_client test/test_service_client.test test/service_client.cpp)
  target_link_libraries(${PROJECT_NAME}_test_service_client ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_service_client PROPERTIES OUTPUT_NAME test_service_client PREFIX "")

  add_executable(${PROJECT_NAME}_test_service_client_services test/service_client_test_services.cpp)
  target_link_libraries(${PROJECT_NAME}_test_service_client_services ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_service_client_services PROPERTIES OUTPUT_NAME test_service_client_services PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_action_client test/test_action_client.test test/action_client.cpp)
  target_link_libraries(${PROJECT_NAME}_test_action_client ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_action_client PROPERTIES OUTPUT_NAME test_action_client PREFIX "")

  add_executable(${PROJECT_NAME}_test_action_client_test_server test/action_client_test_server.cpp)
  target_link_libraries(${PROJECT_NAME}_test_action_client_test_server ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_action_client_test_server PROPERTIES OUTPUT_NAME test_action_client_test_server PREFIX "")
endif ()

# to run: catkin build ros_babel_fish --no-deps -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args ros_babel_fish_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information and code to get the coverage
# Note: Sometimes this command has to be run twice. Don't know why. If you do, please tell me!
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/examples*" "*/${PROJECT_NAME}/benchmarks*")
  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif ()
