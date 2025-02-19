#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// Déclaration globale du publisher et de la trajectoire
ros::Publisher path_pub;
nav_msgs::Path path;

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

    // Définir le repère de la trajectoire
    path.header.frame_id = "map";

    // Boucle ROS
    ros::spin();

    return 0;
}
