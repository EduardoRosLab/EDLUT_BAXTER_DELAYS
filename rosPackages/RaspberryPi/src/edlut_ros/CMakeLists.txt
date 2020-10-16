cmake_minimum_required(VERSION 2.8.3)
project(edlut_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  message_filters
  message_generation
  roscpp
  rospy
  std_msgs
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 Analog.msg
 AnalogCompact.msg
 LearningState.msg
 AnalogCompactDelay.msg
 Time.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 baxter_core_msgs
 std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES edlut_ros
  CATKIN_DEPENDS roscpp rospy std_msgs baxter_core_msgs message_filters message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /home/baxterpi/ros_ws/devel/include
  include
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/edlut_ros.cpp
# )

add_library(baxter_interface
  src/raspberry_PDSupervisor.cpp
  src/TorqueAddition_Delay.cpp
  src/TorqueAddition_Delay_PD_Wifi.cpp
  src/Q_TrajectoryController.cpp
  src/Q_TrajectoryGenerator.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(baxter_interface edlut_ros_generate_messages_cpp)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/edlut_ros_node.cpp)
add_executable(raspberry_pd_supervisor_node src/raspberry_pd_supervisor_node.cpp)
add_executable(torque_final_node_delay src/torque_final_node_delay.cpp)
add_executable(wireless_robot_state src/wireless_robot_state.cpp)
add_executable(baxter_position_command_node src/baxter_position_command_node.cpp)
add_executable(baxter_position_command_node_DelayStats src/baxter_position_command_node_DelayStats.cpp)
add_executable(q_trajectory_generator_node src/q_trajectory_generator_node.cpp)
add_executable(wifi_testNode_raspberry src/wifi_testNode_raspberry.cpp)
add_executable(rtt_emitter src/rtt_emitter.cpp)
add_executable(rtt_receiver src/rtt_receiver.cpp)
add_executable(baxter_torque_command_node_delays src/baxter_torque_command_node_delays.cpp)
add_executable(torque_final_node_delay_PD_Wifi src/torque_final_node_delay_PD_Wifi.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(raspberry_pd_supervisor_node edlut_ros_generate_messages_cpp)
add_dependencies(torque_final_node_delay edlut_ros_generate_messages_cpp)
add_dependencies(wireless_robot_state edlut_ros_generate_messages_cpp)
add_dependencies(baxter_position_command_node edlut_ros_generate_messages_cpp)
add_dependencies(baxter_position_command_node_DelayStats edlut_ros_generate_messages_cpp)
add_dependencies(q_trajectory_generator_node edlut_ros_generate_messages_cpp)
add_dependencies(wifi_testNode_raspberry edlut_ros_generate_messages_cpp)
add_dependencies(rtt_emitter edlut_ros_generate_messages_cpp)
add_dependencies(rtt_receiver edlut_ros_generate_messages_cpp)
add_dependencies(baxter_torque_command_node_delays edlut_ros_generate_messages_cpp)
add_dependencies(torque_final_node_delay_PD_Wifi edlut_ros_generate_messages_cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(raspberry_pd_supervisor_node
  ${catkin_LIBRARIES}
  baxter_interface
)

target_link_libraries(torque_final_node_delay
  ${catkin_LIBRARIES}
  baxter_interface
)

target_link_libraries(wireless_robot_state
  ${catkin_LIBRARIES}
)

target_link_libraries(baxter_position_command_node
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(baxter_position_command_node_DelayStats
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(q_trajectory_generator_node
  ${catkin_LIBRARIES}
  baxter_interface
)

target_link_libraries(wifi_testNode_raspberry
  ${catkin_LIBRARIES}
  baxter_interface
)

target_link_libraries(rtt_emitter
  ${catkin_LIBRARIES}
)

target_link_libraries(rtt_receiver
  ${catkin_LIBRARIES}
)

target_link_libraries(baxter_torque_command_node_delays
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(torque_final_node_delay_PD_Wifi
  ${catkin_LIBRARIES}
  baxter_interface
)

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_edlut_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
