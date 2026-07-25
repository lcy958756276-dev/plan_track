cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  voronoi_layer
  common
  system_config
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS voronoi_layer common system_config
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB path_planner_src_files src/*.cpp)
file(GLOB path_planner_utils_src_files src/utils/*.cpp)
file(GLOB graph_path_planner_src_files src/graph_planner/*.cpp)
file(GLOB sample_path_planner_src_files src/sample_planner/*.cpp)
file(GLOB evolutionary_path_planner_src_files src/evolutionary_planner/*.cpp)

file(GLOB graph_path_planner_hybrid_astar_src_files src/graph_planner/hybrid_astar_planner/*.cpp)

set(astar_polyline_planner_src
  src/graph_planner/astar_polyline_planner.cpp
)
list(REMOVE_ITEM graph_path_planner_src_files
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_planner/astar_polyline_planner.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${path_planner_src_files}
  ${path_planner_utils_src_files}
  ${graph_path_planner_hybrid_astar_src_files}
  ${astar_polyline_planner_src}
  ${graph_path_planner_src_files}
  ${sample_path_planner_src_files}
  ${evolutionary_path_planner_src_files}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
