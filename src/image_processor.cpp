// image_processor.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>

ros::Publisher arret_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher& pub)
{
    try
    {
        // Convertir l'image ROS en format OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);


         // Convertir en niveaux de gris
        cv::Mat hsv;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        // Définir la plage de couleur rouge dans l'espace HSV
        // Plage de rouge pour OpenCV (en HSV)
        cv::Scalar lower_red(0, 120, 70);  // Couleur rouge faible
        cv::Scalar upper_red(10, 255, 255);  // Couleur rouge forte

        // Créer un masque pour détecter la couleur rouge
        cv::Mat mask;
        cv::inRange(hsv, lower_red, upper_red, mask);

        // Affiner la détection avec un peu de flou pour réduire le bruit
        cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);

        // Trouver les contours de la zone rouge détectée
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        bool obstacle_detecte = false;
        std_msgs::Bool stop_msg;
        // Dessiner un rectangle vert autour de chaque objet rouge détecté
        for (size_t i = 0; i < contours.size(); i++)
        {
            // Si le contour est suffisamment grand, on le considère comme un objet
            if (cv::contourArea(contours[i]) > 100)  // Ajuste cette valeur selon la taille de l'objet
            {
                // Trouver un rectangle englobant pour chaque contour
                cv::Rect bounding_box = cv::boundingRect(contours[i]);
                // Dessiner un rectangle vert
                cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);
                obstacle_detecte = true;
                stop_msg.data = obstacle_detecte;
            }
        }
        arret_pub.publish(stop_msg);
        // Afficher l'image avec le rectangle vert
        cv::imshow("Detected Red Object", cv_ptr->image);
        cv::waitKey(1);  // Nécessaire pour que l'image s'affiche dans OpenCV

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image conversion error: %s", e.what());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processor");

    ros::NodeHandle nh;

    // Initialisation de image_transport avec un NodeHandle
    image_transport::ImageTransport it(nh);

    // Declare a publisher
    image_transport::Publisher pub = it.advertise("/processed_image", 1);

    // Subscribe with properly bound callback
    image_transport::Subscriber sub = it.subscribe("/image_to_process", 1,boost::bind(imageCallback, _1, boost::ref(pub)));
    
    arret_pub = nh.advertise<std_msgs::Bool>("/arret_demande", 10);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

    }
}
