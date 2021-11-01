cmake_minimum_required(VERSION 2.8.3)
project(osm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")

# warnings
# if ( CMAKE_COMPILER_IS_GNUCC )
#     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# endif()
# if ( MSVC )
#     set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
# endif()
add_definitions(-w)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  message_filters
  velodyne_pointcloud
  roslib
  grid_map_ros  
  grid_map_core
  inekf_msgs
  planner_msgs
  grid_map_msgs
  rviz_visual_tools
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

find_package(PCL 1.2 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   DEPENDS
Eigen3
  grid_map_ros  
  grid_map_core
  CATKIN_DEPENDS geometry_msgs

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  external/cereal/include
  /usr/local/include/
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)
include_directories(/usr/local/lib)
include_directories(/usr/local/include/)


# add_executable(fake_robot_publisher src/fake_robot_publisher.cpp
#     src/fake_robot.cpp
#     src/utils/plotting.cpp
# )
# add_dependencies(fake_robot_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})
# target_link_libraries(fake_robot_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )


####################################################################################
#### PIPELINE TESTING 
####################################################################################
add_executable(osm_localization 
    src/osm_lidar_node.cpp
    src/OsmLidar.cpp
)
add_dependencies(osm_localization ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(osm_localization ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
####################################################################################
