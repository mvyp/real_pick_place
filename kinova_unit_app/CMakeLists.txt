cmake_minimum_required(VERSION 2.8.3)
project(kinova_unit_app)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  rviz_visual_tools
  message_generation
  std_msgs
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  actionlib
  actionlib_msgs

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetObject.srv
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
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinova_unit_app
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/kinova_unit_app.cpp
# )


# add_executable(object_save src/object_save.cpp)
# add_executable(object_recognition_verification src/object_recognition_verification.cpp)
# add_executable(marker_detect_node src/marker_detect_node.cpp)


# add_dependencies(object_save ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(marker_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(object_recognition_verification ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# target_link_libraries(object_save
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(marker_detect_node
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(object_recognition_verification
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
