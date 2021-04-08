cmake_minimum_required(VERSION 3.0.2)
project(path_smoother)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Debug
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)

set(NLopt_DIR "/home/chelizi/ThirdPartyLib/nlopt/build")
find_package(NLopt REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  costmap_2d
)

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bucket_queue.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamic_voronoi.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/path_smoother.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/nlopt_smoother.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vec2d.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_smooth.cpp
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES path_smoother
 CATKIN_DEPENDS 
  roscpp
  geometry_msgs
  costmap_2d
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

add_library(path_smoother ${SOURCES})
add_dependencies(path_smoother ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_smoother ${catkin_LIBRARIES})
target_link_libraries(path_smoother ${OpenCV_LIBS} ${NLOPT_LIBRARIES})

add_executable(smooth_node ${SOURCES})
add_dependencies(smooth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smooth_node  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${NLOPT_LIBRARIES})
