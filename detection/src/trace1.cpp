#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cmath>  // Pour std::fabs

// Déclaration globale du publisher et de la trajectoire
ros::Publisher path_pub;
nav_msgs::Path path;
ros::Publisher marker_pub;
int mark_x, mark_y;
std::vector<std::pair<int, int>> marked_positions;  // Liste des positions déjà marquées
// Définir un epsilon pour comparer les positions (pour tolérer les erreurs flottantes)
const float EPSILON = 0.1;

bool isPositionMarked(int x, int y) {
    // Vérifier si la position (x, y) est dans la liste des positions marquées
    for (const auto& pos : marked_positions) {
        if (std::fabs(pos.first - x) < EPSILON && std::fabs(pos.second - y) < EPSILON) {
            return true;  // La position a déjà été marquée
        }
    }
    return false;  // La position n'a pas encore été marquée
}

void addMarker()
{
    if (!isPositionMarked(mark_x, mark_y)) {
        // Crée un message de type Marker
        visualization_msgs::Marker marker;
        
        // Définir le type de marqueur, ici une croix (x)
        marker.type = visualization_msgs::Marker::CUBE;
        
        // Définir la forme du marqueur (taille)
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.01; // Une petite épaisseur pour le "X"
        
        // Définir la couleur du marqueur (rouge)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;  // Opacité du marqueur
        
        // Définir la position (x, y)
        marker.pose.position.x = mark_x;
        marker.pose.position.y = mark_y;
        marker.pose.position.z = 0.0;
        
        // Orientation du marqueur (on peut le laisser à 0 si on n'a pas besoin de rotation)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Définir l'id du marqueur (ici on peut définir un id unique pour chaque point)
        marker.id = 0;  // Change-le si tu veux plusieurs marqueurs différents
        marker.action = visualization_msgs::Marker::ADD;  // Action pour ajouter le marqueur

        // Ajoute un frame_id, ici "map" (ou "base_link", si tu veux le cadre du robot)
        marker.header.frame_id = "map";  // Le frame de référence, par exemple "map" ou "base_link"
        marker.header.stamp = ros::Time::now();  // Horodatage du message

        // Publier le marqueur
        marker_pub.publish(marker);
        
        // Ajouter la position à la liste des positions marquées
        marked_positions.push_back(std::make_pair(mark_x, mark_y));
    }
}

void markerCallback(const geometry_msgs::PoseStamped pose_stamped){
    mark_x = pose_stamped.pose.position.x;
    mark_y = pose_stamped.pose.position.y;
}

// Callback de l'odométrie pour mettre à jour la trajectoire
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;

    path.poses.push_back(pose_stamped);
    path.header.stamp = ros::Time::now();

    // Publier la trajectoire mise à jour
    path_pub.publish(path);
}

int main(int argc, char** argv) {
    // Initialisation du nœud
    ros::init(argc, argv, "trace1");
    ros::NodeHandle nh;

    // Initialisation du publisher et du subscriber
    path_pub = nh.advertise<nav_msgs::Path>("/path", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber marker_sub = nh.subscribe("/marker", 10, markerCallback);
    
    // Création du Publisher pour le Marker
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    // Définir le repère de la trajectoire
    path.header.frame_id = "map";

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // Ajouter un point (ou une croix) à la nouvelle position
        addMarker();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
