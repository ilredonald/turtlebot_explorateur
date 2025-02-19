#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

double lin,ang;//parametre de vitesse a transmettre au robot pour le faire bouger

//cré un message a publié sur /cmd_vel contenant les vitesses lineaire et angulaire du robot
geometry_msgs::Twist genererCommande(double linear,double angular)
{
    geometry_msgs::Twist vitesse;



    vitesse.linear.y = vitesse.linear.z = 0;

    vitesse.angular.z = angular;
    vitesse.angular.x = vitesse.angular.y = 0;

    return vitesse;
}

void arretCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM("Bool\n"<<msg->data);
    // Si le message booléen est true, on arrête le robot
    if (msg->data) {
        ROS_INFO("Obstacle détecté, arrêt du robot.");
        lin = 0.0;  // Arrêter le robot
        ang = 0.0;
    } else {
        ROS_INFO("Pas d'obstacle, poursuite de la navigation.");
        //lin = 0.0;  // Continuer à avancer
        //ang = 0.3925;  // Pas de rotation
    }
}

void visitedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM("Bool\n"<<msg->data);
    // Si le message booléen est true, on arrête le robot
    if (msg->data) {
        //ROS_INFO("chemin deja visite.");
        lin = 0.0;  
        ang = 0.5;
    } 
}

void Scancallback(const sensor_msgs::LaserScan scan)
{
    // Définir l'angle de détection pour l'avant du robot
    int start_index = (scan.angle_min + 0.785) / scan.angle_increment; // -45° en radians
    int end_index = scan.ranges.size() + ((scan.angle_min - 0.785) / scan.angle_increment) -1;   // +45° en radians

    // Calculer la distance minimale sur la zone avant
    float min_range_right = 9999.0;
    float min_range_left = 9999.0;

    for (int i = 0; i < start_index; i++)
    {
        if (scan.ranges[i] < min_range_right)
            min_range_right = scan.ranges[i];
    }

    for (int i = end_index; i < scan.ranges.size(); i++)
    {
        if (scan.ranges[i] < min_range_left)
            min_range_left = scan.ranges[i];
        
    }

    if (min_range_right < 0.3 || min_range_left < 0.3)
    {
        ROS_INFO("Obstacle detecte a moins de 0.3m, on tourne !");
        if(min_range_right < 0.3)
           ang = -0.5;  // Tourner
        if(min_range_left < 0.3)
            ang = 0.5;  // Tourner
            lin = 0.0;
    }
    else
    {
        ROS_INFO("Pas d'obstacle, on avance !");
        ang = 0.0;
        lin = 0.2;  // Avancer
    }
}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"move_auto");

    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //ros::Subscriber stop_sub = nh.subscribe("/arret_demande", 10, &arretCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan",100,&Scancallback);
    //ros::Subscriber visited_sub = nh.subscribe("/visited",100,&visitedCallback);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        cmd_vel_pub.publish(genererCommande(lin,ang));

        ros::spinOnce();

        loop_rate.sleep();
    }
        
}