cmake_minimum_required(VERSION 3.0.2)
project(tuto_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs image_transport cv_bridge std_msgs
)


## Définir les répertoires d'inclusion
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Ajouter les exécutables pour chaque nœud
add_executable(image_receiver src/image_receiver.cpp)
add_executable(image_processor src/image_processor.cpp)
add_executable(image_sender src/image_sender.cpp)

add_executable(move_auto src/move_auto.cpp)
add_executable(trace1 src/trace1.cpp)
add_executable(move_to_not_visited src/move_to_not_visited.cpp)

## Lier les exécutables aux bibliothèques
target_link_libraries(image_receiver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(image_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(image_sender
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


target_link_libraries(move_auto
  ${catkin_LIBRARIES}
)

target_link_libraries(move_to_not_visited
  ${catkin_LIBRARIES}
)

target_link_libraries(trace1
  ${catkin_LIBRARIES})