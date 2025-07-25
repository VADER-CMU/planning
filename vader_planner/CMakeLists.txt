cmake_minimum_required(VERSION 2.8.3)
project(vader_planner)

## Compile as C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    message_generation
    geometry_msgs
    std_msgs
    vader_msgs
    tf
    sensor_msgs
    gazebo_msgs
    roscpp
    roslib
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   pose_plan.srv
#   joint_plan.srv
#   exec_plan.srv
#   single_straight_plan.srv
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
  geometry_msgs
  std_msgs
  vader_msgs
  # Or other packages containing msgs
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
#  LIBRARIES xarm_planner
 CATKIN_DEPENDS message_runtime geometry_msgs std_msgs vader_msgs moveit_visual_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/xarm_planner.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/xarm_planner_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_xarm_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(utils src/utils/utils.cpp)
add_dependencies(utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(utils ${catkin_LIBRARIES})
# add_executable(xarm_simple_planner src/xarm_simple_planner.cpp)
# add_dependencies(xarm_simple_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(xarm_simple_planner ${catkin_LIBRARIES})

# add_executable(vader_dual_arm_planner src/vader_dual_arm_planner.cpp)
# add_dependencies(vader_dual_arm_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_dual_arm_planner ${catkin_LIBRARIES})

# add_executable(vader_single_arm_planner src/vader_single_arm_planner.cpp)
# add_dependencies(vader_single_arm_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_single_arm_planner ${catkin_LIBRARIES})

# add_executable(vader_pepper_to_poi src/vader_pepper_to_POI.cpp)
# add_dependencies(vader_pepper_to_poi ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_pepper_to_poi ${catkin_LIBRARIES})

# add_executable(vader_rohit_planner src/vader_single_arm_planner_rohit.cpp)
# add_dependencies(vader_rohit_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_rohit_planner ${catkin_LIBRARIES})

# add_executable(vader_abhi_planner src/vader_dual_arm_planner_coll_avoid_abhi.cpp)
# add_dependencies(vader_abhi_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_abhi_planner ${catkin_LIBRARIES})

# add_executable(vader_PR2_planner src/vader_PR2_planner.cpp)
# add_dependencies(vader_PR2_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_PR2_planner ${catkin_LIBRARIES})

# add_executable(vader_single_arm_planner_PC_withrot src/vader_single_arm_planner_PC_withrot.cpp)
# add_dependencies(vader_single_arm_planner_PC_withrot ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_single_arm_planner_PC_withrot ${catkin_LIBRARIES})


add_executable(vader_svd_dual_arm_planner src/vader_svd_dual_arm_planner.cpp)
add_dependencies(vader_svd_dual_arm_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(vader_svd_dual_arm_planner ${catkin_LIBRARIES})

# add_executable(cutter_logic_test src/cutter_logic_test.cpp)
# add_dependencies(cutter_logic_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(cutter_logic_test ${catkin_LIBRARIES})

# add_executable(vader_single_arm_planner_pgtg src/vader_single_arm_planner_pgtg.cpp)
# add_dependencies(vader_single_arm_planner_pgtg ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vader_single_arm_planner_pgtg ${catkin_LIBRARIES})

add_executable(vader_svd_single_arm_planner src/vader_svd_single_arm_planner.cpp)
add_dependencies(vader_svd_single_arm_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(vader_svd_single_arm_planner ${catkin_LIBRARIES})

#-------------------------------------manual tests-------------------------------------#

add_executable(hello_world src/tests/hello.cpp)
add_dependencies(hello_world ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(hello_world ${catkin_LIBRARIES})

add_executable(collision_object_utils_tests src/tests/collision_object_utils_tests.cpp)
add_dependencies(collision_object_utils_tests ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(collision_object_utils_tests utils ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS
  # xarm_simple_planner
  # xarm_simple_planner_test
  # xarm_gripper_planner
  # vader_single_arm_planner
  # vader_dual_arm_planner
  # vader_pepper_to_poi
  # vader_rohit_planner
  # vader_abhi_planner
  # vader_PR2_planner
  # cutter_logic_test
  # vader_single_arm_planner_pgtg
  # vader_single_arm_planner_PC_withrot
  utils
  collision_object_utils_tests
  vader_svd_dual_arm_planner
  vader_svd_single_arm_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)