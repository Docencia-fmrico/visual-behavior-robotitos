cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  darknet_ros_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  behaviortree_cpp_v3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/visual_behavior/DetectBall.cpp
  src/visual_behavior/DetectPerson.cpp
  src/visual_behavior/DetectPersonDist.cpp
  src/visual_behavior/MakeSound.cpp
  src/visual_behavior/Turn.cpp
  src/visual_behavior/Foward.cpp
)

add_executable(visual_person_node src/visual_person_node.cpp)
target_link_libraries(visual_person_node ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(visual_ball_node src/visual_ball_node.cpp)
target_link_libraries(visual_ball_node ${catkin_LIBRARIES} ${PROJECT_NAME})

roslint_cpp(
    src/visual_person_node.cpp
    src/visual_ball_node.cpp
    include/visual_behavior/DetectPerson.h 
    src/visual_behavior/DetectPerson.cpp
    include/visual_behavior/DetectPersonDist.h 
    src/visual_behavior/DetectPersonDist.cpp
    include/visual_behavior/DetectBall.h 
    src/visual_behavior/DetectBall.cpp
    include/visual_behavior/MakeSound.h 
    src/visual_behavior/MakeSound.cpp
    include/visual_behavior/Turn.h 
    src/visual_behavior/Turn.cpp
    include/visual_behavior/Foward.h 
    src/visual_behavior/Foward.cpp
)
