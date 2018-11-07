cmake_minimum_required(VERSION 2.8.3)
project(gazebo_rti_state_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(NDDSHOME /opt/rti_connext_dds-5.3.1)
set(TARGET_ARCH x64Linux3gcc5.4.0)

add_compile_options(-DRTI_LINUX)
add_compile_options(-DRTI_UNIX)
add_compile_options(-DRTI_64BIT)
add_compile_options(-DRTI_STATIC)
add_compile_options(-DNDDS_STANDALONE_TYPE)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  gazebo_ros_distance_plugin
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)
find_package(fcl)
find_package(Gazebo REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "/opt/rti_connext_dds-5.3.1/resource/cmake")
#find_package(RTIConnextDDS REQUIRED COMPONENTS core)

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
#  LIBRARIES gazebo_rti_state_plugin
#  CATKIN_DEPENDS roscpp
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
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include/
  src/
  ../gazebo_ros_distance_plugin/include/
  /opt/rti_connext_dds-5.3.1/include/ndds/hpp/
  /opt/rti_connext_dds-5.3.1/include/ndds/
  /opt/rti_connext_dds-5.3.1/include/
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/plugin_main.cpp
  #src/gen/GazeboObjectState_publisher.cxx
  #src/gen/GazeboObjectStateImplPlugin.cxx
  src/gen/GazeboObjectState.cxx
  src/gen/GazeboObjectStatePlugin.cxx
  #src/gen/GazeboObjectStateImpl.cxx
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscz.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscorez.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscpp2z.a
   dl
   nsl
   m
   pthread
   rt
)
add_library(${PROJECT_NAME}_distance
  src/plugin_model_rti.cpp
  ../gazebo_ros_distance_plugin/src/plugin_common.cpp
  ../gazebo_ros_distance_plugin/src/distance_calculation.cpp
  #src/gen/GazeboObjectStateImplPlugin.cxx
  src/gen/GazeboObjectState.cxx
  src/gen/GazeboObjectStatePlugin.cxx
  #src/gen/GazeboObjectStateImpl.cxx
)
target_link_libraries(${PROJECT_NAME}_distance
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscz.a
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscorez.a
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscpp2z.a
   dl
   nsl
   m
   pthread
   rt
   fcl 
   ccd
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rti_example_pub_node
  #src/gen/GazeboObjectStateImplPlugin.cxx
  src/gen/GazeboObjectState.cxx
  src/gen/GazeboObjectStatePlugin.cxx
  #src/gen/GazeboObjectStateImpl.cxx
  src/gen/GazeboObjectState_publisher.cxx 
  )
target_link_libraries(rti_example_pub_node 
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscz.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscorez.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscpp2z.a
   dl
   nsl
   m
   pthread
   rt
 )
# Example sub
add_executable(rti_example_sub_node
  #src/gen/GazeboObjectStateImplPlugin.cxx
  src/gen/GazeboObjectState.cxx
  src/gen/GazeboObjectStatePlugin.cxx
  #src/gen/GazeboObjectStateImpl.cxx
  src/gen/GazeboObjectState_subscriber.cxx 
  )
target_link_libraries(rti_example_sub_node 
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscz.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscorez.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscpp2z.a
   dl
   nsl
   m
   pthread
   rt
 )
# Test pub
add_executable(rti_example_publisher
  #src/gen/GazeboObjectStateImplPlugin.cxx
  src/gen/GazeboObjectState.cxx
  src/gen/GazeboObjectStatePlugin.cxx
  #src/gen/GazeboObjectStateImpl.cxx
  src/example_publisher.cpp
  )
target_link_libraries(rti_example_publisher
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscz.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscorez.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscpp2z.a
   dl
   nsl
   m
   pthread
   rt
 )
# Test subscriber
add_executable(rti_example_subscriber
  #src/gen/GazeboObjectStateImplPlugin.cxx
  src/gen/GazeboObjectState.cxx
  src/gen/GazeboObjectStatePlugin.cxx
  #src/gen/GazeboObjectStateImpl.cxx
  src/example_subscriber.cpp
  )
target_link_libraries(rti_example_subscriber
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscz.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscorez.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscpp2z.a
   dl
   nsl
   m
   pthread
   rt
 )
add_executable(laser_min_node 
  #src/laser_min_filter.cpp
  src/gen/GazeboObjectState_publisher.cxx
  src/gen/GazeboObjectState.cxx
  src/gen/GazeboObjectStatePlugin.cxx
)
target_link_libraries(laser_min_node 
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${NDDSHOME}/lib/${TARGET_ARCH}/libnddscz.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscorez.a
   ${NDDSHOME}/lib/${TARGET_ARCH}//libnddscpp2z.a
   dl
   nsl
   m
   pthread
   rt
 )
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gazebo_rti_state_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
