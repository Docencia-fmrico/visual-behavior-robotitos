cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf
  image_transport
  pcl_conversions
  pcl_ros
  behaviortree_cpp_v3
  darknet_ros_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
LIBRARIES geometry_tf
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  cv_bridge
  behaviortree_cpp_v3
  tf
  image_transport
  pcl_conversions
  pcl_ros
  message_filters
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV)
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/visual_behavior/DetectBall.cpp
  src/visual_behavior/DetectBallDist.cpp
  src/visual_behavior/DetectPerson.cpp
  src/visual_behavior/DetectPersonDist.cpp
  src/visual_behavior/MakeSound.cpp
  src/visual_behavior/Turn.cpp
  src/visual_behavior/Foward.cpp
  src/visual_behavior/PID.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_definitions(${PCL_DEFINITIONS})

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(LINK_LIBS
  visual_behavior
  ${catkin_LIBRARIES}
)

add_executable(visual_person_node src/visual_person_node.cpp)
target_link_libraries(visual_person_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${LINK_LIBS})
add_executable(visual_ball_node src/visual_ball_node.cpp)
target_link_libraries(visual_ball_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${LINK_LIBS})
add_executable(visual_complete_node src/visual_complete_node.cpp)
target_link_libraries(visual_complete_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${LINK_LIBS})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")

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
    include/visual_behavior/PID.h 
    src/visual_behavior/PID.cpp
    include/visual_behavior/transforms.h 
    src/visual_behavior/transforms.cpp
)

