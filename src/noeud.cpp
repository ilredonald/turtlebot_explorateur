// Fichier : src/turtle_mover_node.cpp

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    // Initialisation du nœud ROS
    ros::init(argc, argv, "turtle_mover");

    // Création d'un handle pour le node
    ros::NodeHandle n;

    // Création du publisher pour le topic /turtle1/cmd_vel
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // Création du message de type Twist
    geometry_msgs::Twist move_cmd;

    // Définir les valeurs de vitesse linéaire et angulaire
    move_cmd.linear.x = 2.0;  // Vitesse linéaire (avant) sur l'axe X
    move_cmd.angular.z = 1.8; // Vitesse angulaire (rotation autour de l'axe Z)

    // Fréquence de publication (10 Hz)
    ros::Rate loop_rate(10);

    // Publier les messages dans une boucle
    while (ros::ok())
    {
        pub.publish(move_cmd);
        ROS_INFO("Publishing: linear.x = %f, angular.z = %f", move_cmd.linear.x, move_cmd.angular.z);
        loop_rate.sleep();
    }

    return 0;
}

