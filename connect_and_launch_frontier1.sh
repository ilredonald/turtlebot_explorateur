#!/bin/bash

# Demander l'adresse IP du TurtleBot
read -p "Veuillez entrer l'adresse IP du TurtleBot : " TURTLEBOT_IP

# Exporter l'IP du TurtleBot
export TURTLEBOT_IP=$TURTLEBOT_IP

# Sourcer les workspaces
source /opt/ros/noetic/setup.bash
source ~/workspace/devel/setup.bash  # <-- CORRIGÉ depuis ~/catkin_ws/

# Configuration réseau ROS
export ROS_MASTER_URI=http://$TURTLEBOT_IP:11311
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')  # <-- Ton IP locale automatiquement

# Choix du modèle
export TURTLEBOT3_MODEL=burger

# Lancement des différentes parties de l'exploration

# Lancement de SLAM
gnome-terminal -- bash -c "roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping; exec bash"

# Vérifier si SLAM a bien démarré
if [ $? -ne 0 ]; then
    echo "Erreur lors du lancement de SLAM"
    exit 1
fi

# Lancement de move_base
gnome-terminal -- bash -c "roslaunch turtlebot3_navigation move_base.launch; exec bash"

# Vérifier si move_base a bien démarré
if [ $? -ne 0 ]; then
    echo "Erreur lors du lancement de move_base"
    exit 1
fi

# Lancement de frontier_exploration
gnome-terminal -- bash -c "roslaunch frontier_exploration explore_costmap.launch; exec bash"

# Vérifier si frontier_exploration a bien démarré
if [ $? -ne 0 ]; then
    echo "Erreur lors du lancement de frontier_exploration"
    exit 1
fi

# Lancement du noeud de détection
gnome-terminal -- bash -c "roslaunch detection projet.launch; exec bash"

# Vérifier si le noeud de détection a bien démarré
if [ $? -ne 0 ]; then
    echo "Erreur lors du lancement du noeud de détection"
    exit 1
fi

echo "Tous les processus ont été lancés avec succès!"
