cmake_minimum_required(VERSION 2.8.3)
project(online_adjusting)

## Compile as C++11, supported in ROS Kinetic and newer
# 
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  automated_driving_msgs
)
find_package(SDL2 REQUIRED)

# PKG_SEARCH_MODULE(SDL2IMAGE REQUIRED SDL2_image>=2.0.0)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS automated_driving_msgs
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
)

add_library(online_adjusting_lib 
src/online_adjusting.cpp
)

add_executable(online_adjusting_node src/online_adjusting_node.cpp)

add_dependencies(online_adjusting_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(online_adjusting_node online_adjusting_lib
  ${catkin_LIBRARIES}
  ${SDL2_LIBRARIES}
)
