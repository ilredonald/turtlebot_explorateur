read -p "Veuillez entrer l'adresse IP du TurtleBot : " TURTLEBOT_IP
export TURTLEBOT_IP=$TURTLEBOT_IP
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://$TURTLEBOT_IP:11311
export ROS_HOSTNAME=172.16.211.31
export TURTLEBOT3_MODEL=burger

#initialisation de l'algorithme frontier exploration

##initialisation de slam pour le mapping
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
##initialisation de move_base
roslaunch turtlebot3_navigation move_base.launch
##initialisation de frontier_exploration
roslaunch frontier_exploration explore_costmap.launch
