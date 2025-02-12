#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

// Déclaration globale du publisher, de la trajectoire, des positions visitées et du publisher de commande
ros::Publisher path_pub;
ros::Publisher cmd_vel_pub;  // Publisher pour envoyer les commandes de vitesse
nav_msgs::Path path;
std::vector<geometry_msgs::Pose> visited_poses;  // Stocker les poses visitées

// Fonction pour calculer la distance entre deux poses
double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx * dx + dy * dy);  // Calculer la distance Euclidienne 2D
}

// Fonction pour effectuer une rotation
void rotateRobot(double angle) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = angle;  // Rotation autour de l'axe z (vertical)
    cmd_vel_pub.publish(cmd_vel);  // Publier la commande de rotation

    // Attente d'un court instant pour effectuer la rotation
    ros::Duration(2.0).sleep();  // La durée de la rotation peut être ajustée selon la vitesse du robot

    // Arrêter le robot après la rotation
    cmd_vel.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel);
}

// Callback de l'odométrie pour mettre à jour la trajectoire
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;

    // Vérifier si la position est trop proche d'une position déjà visitée
    bool too_close = false;
    double distance_threshold = 0.5;  // Seuil de distance (en mètres)

    for (const auto& visited_pose : visited_poses) {
        double distance = calculateDistance(pose_stamped.pose, visited_pose);
        if (distance < distance_threshold) {
            too_close = true;  // Si la position est trop proche d'une position visitée
            break;
        }
    }
    path.poses.push_back(pose_stamped);
    // Si la position est assez éloignée, on l'ajoute à la trajectoire
    if (!too_close) {
        
        visited_poses.push_back(pose_stamped.pose);  // Ajouter la pose à la liste des positions visitées
        path.header.stamp = ros::Time::now();

        // Publier la trajectoire mise à jour
        path_pub.publish(path);
    } else {
        // Si le robot est sur une trajectoire déjà visitée, effectue une rotation
        ROS_INFO("Position déjà visitée, rotation en cours...");
        rotateRobot(1.0);  // Rotation de 1 radian (peut être ajusté)
    }
}

int main(int argc, char** argv) {
    // Initialisation du nœud
    ros::init(argc, argv, "trace2");
    ros::NodeHandle nh;

    // Initialisation du publisher et du subscriber
    path_pub = nh.advertise<nav_msgs::Path>("/path", 10);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  // Publisher pour la commande de vitesse
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Définir le repère de la trajectoire
    path.header.frame_id = "map";

    // Boucle ROS
    ros::spin();

    return 0;
}
