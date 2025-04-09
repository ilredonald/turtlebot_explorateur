#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


double lin,ang;//parametre de vitesse a transmettre au robot pour le faire bouger
nav_msgs::Path myPath;
sensor_msgs::LaserScan scany;
bool verif = false;
bool route = false;
bool negate = false;
bool turn = false;
nav_msgs::Odometry actuOdom;
nav_msgs::Odometry posedOdom;
geometry_msgs::PoseStamped visitedPose;


//cré un message a publié sur /cmd_vel contenant les vitesses lineaire et angulaire du robot
geometry_msgs::Twist genererCommande(double linear,double angular)
{
    geometry_msgs::Twist vitesse;

    vitesse.linear.x = linear;
    vitesse.linear.y = vitesse.linear.z = 0;

    vitesse.angular.z = angular;
    vitesse.angular.x = vitesse.angular.y = 0;

    return vitesse;
}

double angular(geometry_msgs::PoseStamped& pose1,nav_msgs::Odometry& Odompose)
{
    tf::Quaternion q1(
        pose1.pose.orientation.x,
        pose1.pose.orientation.y,
        pose1.pose.orientation.z,
        pose1.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll1, pitch1, yaw1;
    //permet d'avoir les differents angles par rapport aux axes pour notre position actuelle
    m1.getRPY(roll1,pitch1,yaw1);

    tf::Quaternion q2(
        Odompose.pose.pose.orientation.x,
        Odompose.pose.pose.orientation.y,
        Odompose.pose.pose.orientation.z,
        Odompose.pose.pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    double roll2, pitch2, yaw2;
    //permet d'avoir les differents angles par rapport aux axes pour une position de la liste
    m2.getRPY(roll2,pitch2,yaw2);

    return yaw1 - yaw2;
}

void jeBouge()
{
    if(!route)
    {
        // Définir l'angle de détection pour l'avant du robot
        int start_index = (scany.angle_min + 0.3925) / scany.angle_increment; // -45°/2 en radians
        int end_index = scany.ranges.size() + ((scany.angle_min - 0.3925) / scany.angle_increment) -1;   // +45°/2 en radians

        // Calculer la distance minimale sur la zone avant
        float min_range_right = 9999.0;
        float min_range_left = 9999.0;

        for (int i = 0; i < start_index; i++)
        {
            if (scany.ranges[i] < min_range_right)
                min_range_right = scany.ranges[i];
        }

        for (int i = end_index; i < scany.ranges.size(); i++)
        {
            if (scany.ranges[i] < min_range_left)
                min_range_left = scany.ranges[i];
        
        }

        //Si on detecte un obstacle on tourne en fonction de la position de ce dernier
        if (min_range_right < 0.25 || min_range_left < 0.25)
        {
            ROS_INFO("Obstacle detecte a moins de 0.25m, on tourne !");
            if(!turn)
            {
                if(min_range_right < 0.25)
                    ang = -0.3925;  // Tourner a gauche si l'obstacle est a droite
                if(min_range_left < 0.25)
                    ang = 0.3925;  // Tourner a droite si l'obstacle est a gauche
                lin = 0.0;
                turn = true;//Signaler qu'on est entrain de tourner
            }
        }
        else//si il n'y a pas d'obstacles avancer
        {
            ROS_INFO("Pas d'obstacle, on avance !");
            ang = 0.0;
            lin = 0.1;  // Avancer
            turn = false;
        }
    }
    else
    {//si l'angle entre la position visite et la position marquee est positif et qu'il ny a pas d'action "negative" en cours
        if(angular(visitedPose,posedOdom)>0 && !negate)
        {
            ROS_INFO("Positif alors, je sais ou je dois aller !");
            ang = -0.3925; 
            lin = 0.0;
            if(abs(angular(visitedPose,actuOdom))>=0.3925)//tourner jusqu'a ce qu'on face au moins un angle de pi/8
            {
                ang = 0;
                route = false;//signaler qu'on a fini le changement de trajectoire pour explorer une nouvelle zone
            }
        }
        else if(negate)
        {
            ROS_INFO("Negatif alors, je sais ou je dois aller !");
            ang = 0.3925;
            lin = 0.0;
            if(abs(angular(visitedPose,actuOdom))>=0.3925)//tourner jusqu'a ce qu'on face au moins un angle de pi/8
            {
                ang = 0;
                route = false;//signaler qu'on a fini le changement de trajectoire pour explorer une nouvelle zone
                negate = false;//signaler qu'on a fini l'action negative en cours
            }
        }
    }
}

void Scancallback(const sensor_msgs::LaserScan scan)
{
    scany = scan;//recuperer le scan actuel

    jeBouge();
}

void Pathcallback(const nav_msgs::Path& path)
{
    myPath = path;//recuperer le chemin (le path)
    if(myPath.poses.size()>1000)
        verif = true;//signaler qu'on a assez d'elements dans le tableau 
}

void Odomcallback(const nav_msgs::Odometry& odom)
{
    if(verif)
    {
        int i = 0;
        //supprimer les derniers elements pour que le robot puisse se deplacer
        while(i!=500)
        {
            myPath.poses.pop_back();
            i++;
        }
        
    }
    

    for(geometry_msgs::PoseStamped& pose : myPath.poses)
    {
        actuOdom = odom;//recuperer l'odometrie actuelle
        if(verif)
            if(abs(pose.pose.position.x - odom.pose.pose.position.x)<0.2 && abs(pose.pose.position.y - odom.pose.pose.position.y)<0.2)
            {
                //calcul de l'angle entre une position de la liste et la position actuelle
                if(abs(angular(pose,actuOdom))<0.19625)
                {
                    ROS_INFO("Hum... C'est comme xi je suis deja passe par la moi...\nChangeons de route voir");
                    posedOdom = odom;//enregistrer la position a changer
                    visitedPose = pose;//enregistrer la position avec laquelle on la compare
                    route = true;//signaler qu'on va changer de route pour explorer de nouveau horizons
                    if(angular(pose,actuOdom)<0)
                        negate = true;//si l'angle est negatif, signaler qu'on va entamer une action "negative"
                    
                    jeBouge();
                }
            }
    }
}



int main(int argc,char ** argv)
{
    ros::init(argc,argv,"move_auto");

    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Subscriber scan_sub = nh.subscribe("/scan",100,&Scancallback);
    ros::Subscriber path_sub = nh.subscribe("/path",100,&Pathcallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom",100,&Odomcallback);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        cmd_vel_pub.publish(genererCommande(lin,ang));//donner les vitesses du robot

        ros::spinOnce();

        loop_rate.sleep();
    }
        
}