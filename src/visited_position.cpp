#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

bool verif = false;
bool route = false;
bool negate = false;
nav_msgs::Odometry actuOdom;
nav_msgs::Odometry posedOdom;
geometry_msgs::PoseStamped visitedPose;
nav_msgs::Path myPath;

// Calcul de l'angle entre deux positions
double angular(geometry_msgs::PoseStamped& pose1, nav_msgs::Odometry& Odompose) {
    tf::Quaternion q1(
        pose1.pose.orientation.x,
        pose1.pose.orientation.y,
        pose1.pose.orientation.z,
        pose1.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll1, pitch1, yaw1;
    m1.getRPY(roll1, pitch1, yaw1);

    tf::Quaternion q2(
        Odompose.pose.pose.orientation.x,
        Odompose.pose.pose.orientation.y,
        Odompose.pose.pose.orientation.z,
        Odompose.pose.pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    double roll2, pitch2, yaw2;
    m2.getRPY(roll2, pitch2, yaw2);

    return yaw1 - yaw2;
}

// Fonction de gestion du changement de route si un point a déjà été visité
void jeBouge() {
    if (route) {
        if (angular(visitedPose, posedOdom) > 0 && !negate) {
            ROS_INFO("Positif alors, je sais où je dois aller !");
            // Logique pour tourner dans une direction
            route = false;
        } else if (negate) {
            ROS_INFO("Négatif alors, je sais où je dois aller !");
            // Logique pour tourner dans l'autre direction
            route = false;
            negate = false;
        }
    }
}

// Callback pour recevoir le path et vérifier les points déjà visités
void Pathcallback(const nav_msgs::Path& path) {
    myPath = path;
    if (myPath.poses.size() > 1000)
        verif = true;
}

// Callback pour recevoir les données d'odométrie
void Odomcallback(const nav_msgs::Odometry& odom) {
    if (verif) {
        int i = 0;
        while (i != 500) {
            myPath.poses.pop_back();
            i++;
        }
    }

    for (geometry_msgs::PoseStamped& pose : myPath.poses) {
        actuOdom = odom;
        if (verif && abs(pose.pose.position.x - odom.pose.pose.position.x) < 0.3 && abs(pose.pose.position.y - odom.pose.pose.position.y) < 0.3) {
            if (abs(angular(pose, actuOdom)) < 0.3925) {
                ROS_INFO("Hum... C'est comme si j'étais déjà passé par là...");
                posedOdom = odom;
                visitedPose = pose;
                route = true;
                if (angular(pose, actuOdom) < 0)
                    negate = true;
                jeBouge();
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "visited_position");
    ros::NodeHandle nh;

    ros::Subscriber path_sub = nh.subscribe("/path", 100, &Pathcallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 100, &Odomcallback);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
