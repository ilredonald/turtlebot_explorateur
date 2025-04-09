cmake_minimum_required(VERSION 3.0.2)
project(detection)

# Trouver les packages nécessaires
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

# Trouver OpenCV
find_package(OpenCV REQUIRED)

find_library(ZBAR_LIBRARIES NAMES zbar)
include_directories(/usr/include/zbar)

# Déclaration du package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
)

# Inclure les répertoires de vos dépendances
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

# Ajouter les exécutables
add_executable(traitement_image_node src/traitement_image.cpp)
add_executable(qrcode_node src/qrcode.cpp) 
add_executable(ra2_node src/ra2.cpp)
add_executable(objectrecognition_node src/objectrecognition.cpp)  # Ajout du noeud objectrecognition

# Lier les exécutables avec les bibliothèques nécessaires
target_link_libraries(traitement_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(qrcode_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)

target_link_libraries(ra2_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(objectrecognition_node  # Lien avec les bibliothèques nécessaires
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

