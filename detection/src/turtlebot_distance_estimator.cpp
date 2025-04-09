#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

#define FOCALE_PX 627       // Focale en pixels
#define HAUTEUR_OBJET 1.65 
#define LARGEUR_IMAGE 640   
#define HAUTEUR_IMAGE 480

class ImageAnnotator {
public:
    ImageAnnotator(ros::NodeHandle& nh) {
        // Initialiser le gestionnaire de nœud et les abonnements
        image_sub_ = nh.subscribe("/camera/image_squared", 1, &ImageAnnotator::imageCallback, this);
        image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/image_annotated", 1);
        marker_pub = nh.advertise<geometry_msgs::PoseStamped>("/marker", 1);
        odom_sub_ = nh.subscribe("/odom", 1, &ImageAnnotator::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_orientation_ = tf::getYaw(msg->pose.pose.orientation);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Convertir l'image ROS en image OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Convertir l'image en espace de couleur HSV (meilleur pour détecter la couleur)
            cv::Mat hsv_image;
            cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

            // Définir les plages de couleur pour détecter le vert (couleur du carré)
            cv::Scalar lower_green(35, 50, 50);  // Plage inférieure pour le vert
            cv::Scalar upper_green(85, 255, 255); // Plage supérieure pour le vert

            // Créer un masque qui isole la couleur verte
            cv::Mat green_mask;
            cv::inRange(hsv_image, lower_green, upper_green, green_mask);

            // Appliquer une érosion et une dilatation pour combler les trous et renforcer les contours
            cv::Mat erode_mask, dilate_mask;
            int erosion_size = 3;  // Taille du noyau d'érosion
            int dilation_size = 5;  // Taille du noyau de dilatation

            // Appliquer l'érosion
            cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erosion_size, erosion_size));
            cv::erode(green_mask, erode_mask, element_erode);

            // Appliquer la dilatation sur l'image érodée pour combler les trous
            cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilation_size, dilation_size));
            cv::dilate(erode_mask, dilate_mask, element_dilate);

            // Trouver les contours du carré vert détecté
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(dilate_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (contours.empty()) {
                ROS_WARN("Aucun carré vert trouvé.");
                return;
            }

            // Fusionner les contours pour obtenir une boîte englobante complète du robot
            cv::Rect bounding_box = cv::boundingRect(contours[0]);
            for (size_t i = 1; i < contours.size(); i++) {
                bounding_box = bounding_box | cv::boundingRect(contours[i]);  // Fusionner les boîtes
            }

            // Dessiner la boîte englobante autour de l'objet
            cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);

            // Calculer la distance avec la formule de Thalès
            float hauteur_objet_image = bounding_box.height;  // Hauteur de l'objet détecté en pixels
            float distance = (FOCALE_PX * HAUTEUR_OBJET) / hauteur_objet_image;  // Distance en mètres

            // Calcul de l'angle (orientation)
            float centre_objet_x = bounding_box.x + bounding_box.width / 2; // X du centre de l'objet
            float centre_image_x = LARGEUR_IMAGE / 2;  // X du centre de l'image

            // Calcul de l'angle en pixels par rapport au centre de l'image
            float angle_pixels = centre_objet_x - centre_image_x;

            // Calcul de l'angle en radians (si on suppose un champ de vue de 90° ou π/2 radians pour toute la largeur de l'image)
            float angle_radians = (angle_pixels / LARGEUR_IMAGE) * (M_PI / 2); // π/2 radians pour 90°

            // Calcul des coordonnées x, y dans le repère du robot
            float x = distance * sin(angle_radians);  // Coordonnée x
            float y = distance * cos(angle_radians);  // Coordonnée y
            
            // Afficher la distance sur l'image
            std::string distance_text = "Distance: " + std::to_string(distance) + " m";
            std::string angle_text = "Angle: " + std::to_string(angle_radians) + " rad";
            std::string coordinates_text = "X: " + std::to_string(x) + ", Y: " + std::to_string(y);

            cv::putText(cv_ptr->image, distance_text, cv::Point(bounding_box.x, bounding_box.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            cv::putText(cv_ptr->image, angle_text, cv::Point(bounding_box.x, bounding_box.y - 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            cv::putText(cv_ptr->image, coordinates_text, cv::Point(bounding_box.x, bounding_box.y - 50),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            // Publier l'image annotée
            sensor_msgs::ImagePtr output_msg = cv_bridge::toCvCopy(msg, "bgr8")->toImageMsg();
            image_pub_.publish(output_msg);

            // Calcul de la position de l'objet par rapport à l'origine de l'odom
            float object_x_global = robot_x_ + x * cos(robot_orientation_) - y * sin(robot_orientation_);
            float object_y_global = robot_y_ + x * sin(robot_orientation_) + y * cos(robot_orientation_);

            // Afficher les coordonnées globales de l'objet
            ROS_INFO("Position de l'objet dans le repère odom : x = %f, y = %f", object_x_global, object_y_global);
            
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.pose.position.x = object_x_global;
            pose_stamped.pose.position.y = object_y_global;
            
            marker_pub.publish(pose_stamped);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Erreur cv_bridge : %s", e.what());
        }
    }

private:
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub_;
    
    // Variables pour la position du robot
    float robot_x_;
    float robot_y_;
    float robot_orientation_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_annotator");
    ros::NodeHandle nh;

    ImageAnnotator annotator(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
