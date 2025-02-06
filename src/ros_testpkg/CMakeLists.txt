cmake_minimum_required(VERSION 3.0.2)
project(ros_testpkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  turtlesim
)

catkin_package()

include_directories(
  src/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_driver_node 
  src/main.cpp
  src/turtle_driver.cpp
  src/task_manager.cpp  
  src/PID.cpp

)
add_executable(realistic_drive 
  src/realistic_drive.cpp
)

add_executable(real_main 
  src/real_main.cpp
  src/turtle_driver.cpp
  src/task_manager.cpp
  src/PID.cpp
)

target_link_libraries(turtle_driver_node 
  ${catkin_LIBRARIES}
)

target_link_libraries(realistic_drive 
  ${catkin_LIBRARIES}
)

target_link_libraries(real_main 
  ${catkin_LIBRARIES}
)