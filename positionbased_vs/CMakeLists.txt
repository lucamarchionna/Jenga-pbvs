cmake_minimum_required(VERSION 3.0.2)
project(positionbased_vs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED) 
find_package(VISP REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  visp_ros REQUIRED
  Eigen3 REQUIRED
  VISP REQUIRED
  message_generation
  eigen_conversions
  geometry_msgs
  moveit_ros_planning_interface 
  moveit_ros_move_group
  pcl_ros
  pcl_conversions
  pcl_msgs
  roscpp
  rospy
  visp_ros
  visp_bridge
  tf2
  tf2_ros
  tf2_eigen
)

add_service_files(
  FILES
  InitialGuess.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

add_definitions(${VISP_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${visp_ros_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs pcl_ros pcl_msgs pcl_conversions
  DEPENDS Eigen3 PCL VISP visp_bridge Boost
)


link_directories(${PCL_LIBRARY_DIRS})
# PBVS
add_executable(pbvs
  src/pbvs.cpp
)

target_link_libraries(pbvs
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${VISP_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(pbvs
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(ibvs
  src/ibvs_class.cpp
)

target_link_libraries(ibvs
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${VISP_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(ibvs
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# tracker
add_executable(trackerv1
  src/tracker.cpp
)

target_link_libraries(trackerv1
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(tracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# singleBlock_controlLaw
add_executable(single_block_control
  src/singleBlock_controlLaw.cpp
)

target_link_libraries(single_block_control
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
)

add_dependencies(single_block_control
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# PBVS_sim
add_executable(pbvs_sim
  src/pbvs_sim.cpp
)

target_link_libraries(pbvs_sim
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${VISP_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(pbvs_sim
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# PBVS_class
add_executable(posbased_vs
  src/pbvs_class.cpp
)

target_link_libraries(posbased_vs
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${VISP_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(posbased_vs
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(tutorial-ros-grabber src/tutorial-ros-grabber.cpp)

target_link_libraries(tutorial-ros-grabber ${visp_ros_LIBRARIES})



