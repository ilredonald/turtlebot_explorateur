// image_sender.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Convertir l'image ROS en format OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        // Afficher l'image traitée
        cv::imshow("Processed Image", cv_ptr->image);
        cv::waitKey(1);

        // Si tu veux renvoyer des informations sur l'objet, tu pourrais publier un message sur un autre topic
        // Par exemple, publier la position de l'objet détecté
        // Exemple avec une position fictive
        ROS_INFO("Objet détecté à la position x: 150, y: 150");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Erreur de conversion d'image : %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_sender");
    ros::NodeHandle nh;

    // Souscrire au topic de l'image traitée
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/processed_image", 1, imageCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

    }
}
