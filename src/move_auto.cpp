#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

double lin, ang;
sensor_msgs::LaserScan scany;
bool turn = false;

// Créer un message pour publier les vitesses linéaires et angulaires
geometry_msgs::Twist genererCommande(double linear, double angular) {
    geometry_msgs::Twist vitesse;
    vitesse.linear.x = linear;
    vitesse.linear.y = vitesse.linear.z = 0;
    vitesse.angular.z = angular;
    vitesse.angular.x = vitesse.angular.y = 0;
    return vitesse;
}

// Fonction de déplacement du robot en fonction des obstacles
void jeBouge() {
    int start_index = (scany.angle_min + 0.785) / scany.angle_increment;
    int end_index = scany.ranges.size() + ((scany.angle_min - 0.785) / scany.angle_increment) - 1;

    float min_range_right = 9999.0;
    float min_range_left = 9999.0;

    // Détection des obstacles à droite et à gauche
    for (int i = 0; i < start_index; i++) {
        if (scany.ranges[i] < min_range_right)
            min_range_right = scany.ranges[i];
    }
    for (int i = end_index; i < scany.ranges.size(); i++) {
        if (scany.ranges[i] < min_range_left)
            min_range_left = scany.ranges[i];
    }

    // Si un obstacle est détecté, tourner
    if (min_range_right < 0.25 || min_range_left < 0.25) {
        ROS_INFO("Obstacle détecté à moins de 0.25m, on tourne !");
        if (!turn) {
            if (min_range_right < 0.25)
                ang = -0.3925;  // Tourner à gauche
            if (min_range_left < 0.25)
                ang = 0.3925;   // Tourner à droite
            lin = 0.0;  // Arrêter le robot pour tourner
            turn = true;
        }
    } else {
        ROS_INFO("Pas d'obstacle, on avance !");
        ang = 0.0;
        lin = 0.2;  // Avancer
        turn = false;
    }
}

// Callback pour recevoir les données du scanner laser
void Scancallback(const sensor_msgs::LaserScan& scan) {
    scany = scan;
    jeBouge();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_with_obstacle_avoidance");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 100, &Scancallback);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        cmd_vel_pub.publish(genererCommande(lin, ang));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
