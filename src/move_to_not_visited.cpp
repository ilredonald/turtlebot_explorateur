#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <set>
#include <cmath>
#include <std_msgs/Bool.h>

bool first = true;
// Structure pour représenter les positions visitées
struct Position {
    double x;
    double y;
    Position(double x_val, double y_val) : x(x_val), y(y_val) {}

    // Comparaison standard basée sur x et y
    bool operator<(const Position& other) const {
        if (x == other.x) {
            return y < other.y;  // Comparaison par défaut sur x et y
        }
        return x < other.x;
    }
};

ros::Publisher cmd_vel_pub;
ros::Publisher visited_pub;
std_msgs::Bool vst_msg;
std::set<Position> visited_positions;  // Ensemble pour stocker les positions visitées

// Fonction pour comparer si une position a déjà été visitée avec une certaine tolérance
bool isPositionVisited(const Position& current_position) {
    const double tolerance = 0.05;  // Seuil de tolérance pour la comparaison des positions
    for (const auto& pos : visited_positions) {
        double distance = std::sqrt(std::pow(current_position.x - pos.x, 2) + std::pow(current_position.y - pos.y, 2));
        if (distance < tolerance) {
            return true;  // Position déjà visitée si la distance est inférieure au seuil
        }
    }
    return false;
}

// Fonction de rappel pour l'odométrie
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    Position current_position(x, y);

    // Débogage : Affichage de la position actuelle
    ROS_INFO("Position actuelle: x = %f, y = %f", x, y);
    
    
    // Vérifier si la position a déjà été visitée
    if (!isPositionVisited(current_position) || first) {
        // Nouveau chemin
        visited_positions.insert(current_position);
        ROS_INFO("Nouveau chemin!");
        
        // Publier un message indiquant que le chemin est nouveau
        vst_msg.data = false; // Chemin non visité
        visited_pub.publish(vst_msg);
        first = false;
    } else {
        // Chemin déjà visité
        ROS_INFO("Chemin déjà visité.");
        
        // Publier un message indiquant que le chemin a été visité
        vst_msg.data = true; // Chemin déjà visité
        visited_pub.publish(vst_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_to_not_visited");
    ros::NodeHandle nh;

    // Initialiser les abonnements et publications
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    visited_pub = nh.advertise<std_msgs::Bool>("/visited", 10);
    visited_positions.clear();
    // Définir la fréquence de publication
    ros::Rate rate(10);

    while (ros::ok()) {
        // Traiter les messages entrants
        ros::spinOnce();

        // Maintenir le rythme
        rate.sleep();
    }

    return 0;
}
