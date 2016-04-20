cmake_minimum_required(VERSION 2.8.3)
project(sphero_color)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(sphero_color src/sphero_color.cpp)
target_link_libraries(sphero_color ${catkin_LIBRARIES})
target_link_libraries(sphero_color ${OpenCV_LIBRARIES})
