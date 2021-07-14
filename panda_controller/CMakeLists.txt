cmake_minimum_required(VERSION 3.0.2)
project(panda_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mujoco_ros
  eigen_conversions
)


catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(panda_controller_lib
  src/controller.cpp
  src/mujoco_interface.cpp
)
add_executable(panda_controller src/main.cpp)
target_link_libraries(panda_controller_lib
  ${catkin_LIBRARIES}
  -lrbdl
  -lrbdl_urdfreader
)


target_link_libraries(panda_controller
  panda_controller_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(panda_controller_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(panda_controller ${catkin_EXPORTED_TARGETS})