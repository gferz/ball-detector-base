cmake_minimum_required(VERSION 3.0.2)
project(ball_detector_base)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/ball_detector.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

