cmake_minimum_required(VERSION 3.10)
project(roport)

# Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  hardware_interface
  controller_manager
  behaviortree_cpp_v3
  )

# Optionally build with non ROS pre-installed msgs
find_package(franka_core_msgs)

if (${franka_core_msgs_FOUND})
  include_directories(${franka_core_msgs_INCLUDE_DIRS})
  add_definitions(-DFRANKA_CORE_MSGS)
  MESSAGE(STATUS "FOUND franka_core_msgs")
else (${franka_core_msgs_FOUND})
  MESSAGE(STATUS "NOT FOUND franka_core_msgs")
endif (${franka_core_msgs_FOUND})

find_package(ubt_core_msgs)

if (${ubt_core_msgs_FOUND})
  include_directories(${ubt_core_msgs_INCLUDE_DIRS})
  add_definitions(-DUBT_CORE_MSGS)
  MESSAGE(STATUS "FOUND ubt_core_msgs")
else (${ubt_core_msgs_FOUND})
  MESSAGE(STATUS "NOT FOUND ubt_core_msgs")
endif (${ubt_core_msgs_FOUND})

# Optionally locate Humanoid Path Planner
find_library(HPP_CORE_LIB hpp-core HINTS /opt/openrobots/lib)
if (HPP_CORE_LIB)
  include_directories(/opt/openrobots/include)
  message(STATUS "Found HPP_CORE_LIB ${HPP_CORE_LIB}")
else ()
  message(WARNING "Not Found HPP_CORE_LIB")
endif ()

find_library(HPP_FCL_LIB hpp-fcl HINTS /opt/openrobots/lib)
if (HPP_FCL_LIB)
  message(STATUS "Found HPP_FCL_LIB ${HPP_FCL_LIB}")
else ()
  message(WARNING "Not Found HPP_FCL_LIB")
endif ()

# Optionally locate CartesI/O
find_library(CARTESIO_LIB CartesianInterface HINTS /opt/xbot/)
if (CARTESIO_LIB)
  message(STATUS "Found CARTESIO_LIB ${CARTESIO_LIB}")
  find_package(OpenSoT REQUIRED)
  find_package(cartesian_interface REQUIRED)
else ()
  message(WARNING "CARTESIO_LIB not found")
endif ()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)

###########################################
## Generate services in the 'srv' folder ##
###########################################

add_service_files(
  FILES
  ExecuteAddCollisionBox.srv
  ExecuteAddCollisionPlane.srv
  ExecuteAllPlans.srv
  ExecuteAllPoses.srv
  ExecuteAttachCollisionBox.srv
  ExecuteBinaryAction.srv
  ExecuteDetachCollision.srv
  ExecuteEllipticalTrajectory.srv
  ExecuteFrankaGripperGrasp.srv
  ExecuteGroupJointStates.srv
  ExecuteGroupNamedStates.srv
  ExecuteGroupPlan.srv
  ExecuteGroupPose.srv
  ExecuteGroupPosition.srv
  ExecuteGroupShift.srv
  ExecuteJointPosition.srv
  ExecuteManipulationPlanning.srv
  ExecuteMirroredPose.srv
  ExecutePathPlanning.srv
  ExecuteRemoveCollision.srv
  GetAllNames.srv
  GetGroupJointStates.srv
  GetGroupPose.srv
  GetImageData.srv
  GetPreparePose.srv
  GetTransformedPose.srv
  SaveImage.srv
  SaveJointState.srv
  SaveOdometry.srv
  SavePose.srv
  SenseManipulationPoses.srv
  VisualizePose.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
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

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

add_subdirectory(src/lib/ruckig)

include_directories(
  include
  src/lib/ruckig/include
  ${catkin_INCLUDE_DIRS}
)

###########################
## Declare C++ libraries ##
###########################

add_library(${PROJECT_NAME}
  src/lib/rosout_logger.cpp
  src/lib/robot_interface.cpp
  src/lib/moveit_cpp_server.cpp
  src/lib/msg_converter.cpp
  src/lib/msg_aggregator.cpp
  src/lib/online_trajectory_optimizer.cpp
  )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  rotools_ruckig
  )

if (HPP_CORE_LIB AND HPP_FCL_LIB)
  # These two definitions solve the pinocchio import issue
  add_definitions(-DPINOCCHIO_WITH_HPP_FCL)
  ADD_DEFINITIONS(-DBOOST_MPL_LIMIT_LIST_SIZE=30)

  include_directories(/opt/openrobots/include /usr/local/include)

  find_library(PINOCCHIO_LIBRARIES pinocchio HINTS /usr/local/lib)
  find_library(OMNI_THREAD_LIB omnithread HINTS /opt/openrobots/lib)
  find_library(OMNI_ORB4_LIB omniORB4 HINTS /opt/openrobots/lib)

  add_library(${PROJECT_NAME}_HPP_ADDON
    src/lib/hpp_interface.cpp
    )
  target_link_libraries(${PROJECT_NAME}_HPP_ADDON
    ${catkin_LIBRARIES}
    ${HPP_CORE_LIB}
    ${HPP_FCL_LIB}
    ${OMNI_THREAD_LIB}
    ${OMNI_ORB4_LIB}
    ${PINOCCHIO_LIBRARIES}
    )
  add_dependencies(${PROJECT_NAME}_HPP_ADDON ${PROJECT_NAME}_generate_messages_cpp)
endif ()

if (CARTESIO_LIB)
  include_directories(/opt/xbot/include ${OpenSoT_INCLUDE_DIRS})
  add_library(${PROJECT_NAME}_cartesio src/lib/cartesio_server.cpp)
  target_link_libraries(${PROJECT_NAME}_cartesio ${catkin_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_cartesio ${PROJECT_NAME}_generate_messages_cpp)

  add_library(${PROJECT_NAME}_cartesio_addon src/lib/cartesio_addon.cpp)
  add_dependencies(${PROJECT_NAME}_cartesio_addon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_cartesio_addon
    ${catkin_LIBRARIES}
    ${OpenSoT_LIBRARIES}
    ${cartesian_interface_LIBRARIES})
endif ()

#####################
## Add executables ##
#####################

add_executable(roport_task_scheduler src/roport_task_scheduler.cpp)
add_dependencies(roport_task_scheduler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_task_scheduler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

add_executable(roport_robot_interface src/roport_robot_interface.cpp)
add_dependencies(roport_robot_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_robot_interface
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

add_executable(roport_moveit_cpp_server src/roport_moveit_cpp_server.cpp)
add_dependencies(roport_moveit_cpp_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_moveit_cpp_server
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

add_executable(roport_msg_converter src/roport_msg_converter.cpp)
add_dependencies(roport_msg_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_msg_converter
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

add_executable(roport_msg_aggregator src/roport_msg_aggregator.cpp)
add_dependencies(roport_msg_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_msg_aggregator
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

if (HPP_CORE_LIB AND HPP_FCL_LIB)
  add_executable(roport_hpp_interface src/roport_hpp_interface.cpp)
  add_dependencies(roport_hpp_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(roport_hpp_interface
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${PROJECT_NAME}_HPP_ADDON
    )
endif ()

if (CARTESIO_LIB)
  add_executable(roport_cartesio_server src/roport_cartesio_server.cpp)
  add_dependencies(roport_cartesio_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(roport_cartesio_server
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${PROJECT_NAME}_cartesio
    )
endif ()

############
# Install ##
############

file(GLOB_RECURSE PY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
catkin_install_python(PROGRAMS
  ${PY_SOURCES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  launch
  tree
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_curiosity_frankx_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by python nosetests
catkin_add_nosetests(src/rotools/tests)

##################
## Format check ##
##################

include(${CMAKE_CURRENT_LIST_DIR}/cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
  )
if (CLANG_TOOLS)
  file(GLOB CPP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/*.cpp
    )
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    )
  add_format_target(rotools FILES ${CPP_SOURCES} ${HEADERS})
  add_tidy_target(rotools
    FILES ${CPP_SOURCES}
    DEPENDS ${PROJECT_NAME}
    )
endif ()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
  )
if (PEP_TOOLS)
  file(GLOB_RECURSE PY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(rotools FILES ${PY_SOURCES})
endif ()
