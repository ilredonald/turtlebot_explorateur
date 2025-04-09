#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// Déclaration du publisher pour l'image annotée
ros::Publisher image_pub;

// Callback ROS pour traiter les messages d'image entrants
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // Convertir l'image ROS en image OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Erreur CV Bridge : %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;

    // Charger le modèle YOLO pré-entraîné
    std::string modelConfiguration = "/home/ubuntu/workspace/src/turtlebot32/model/yolov3.cfg";
    std::string modelWeights = "/home/ubuntu/workspace/src/turtlebot32/model/yolov3.weights";
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);

    // Convertir l'image en un blob
    cv::Mat blob = cv::dnn::blobFromImage(img, 1.0/255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);

    net.setInput(blob);
    std::vector<cv::Mat> outs;
    net.forward(outs, net.getUnconnectedOutLayersNames());

    // Traiter les sorties (détections)
    for (size_t i = 0; i < outs.size(); ++i) {
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j) {
            cv::Mat detection = outs[i].row(j);
            float confidence = detection.at<float>(5);

            if (confidence > 0.6) {
                // Obtenir l'ID de la classe et les coordonnées de la boîte englobante
                int class_id = (int)detection.at<float>(0);
                int x_center = (int)(detection.at<float>(0) * img.cols);
                int y_center = (int)(detection.at<float>(1) * img.rows);
                int width = (int)(detection.at<float>(2) * img.cols);
                int height = (int)(detection.at<float>(3) * img.rows);

                // Dessiner la boîte englobante
                cv::rectangle(img, cv::Point(x_center - width / 2, y_center - height / 2),
                              cv::Point(x_center + width / 2, y_center + height / 2),
                              cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    // Afficher l'image avec les boîtes englobantes
    cv::imshow("Object Detection", img);
    cv::waitKey(1);
    cv_ptr->image = img;

    // Publier l'image annotée
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
    image_pub.publish(output_msg);
}

int main(int argc, char **argv) {
    // Initialisation du nœud ROS
    ros::init(argc, argv, "object_detection_node");
    ros::NodeHandle nh;

    // Définir l'abonnement à l'image
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/image", 1, imageCallback);
    image_pub = nh.advertise<sensor_msgs::Image>("/camera/image_squared", 1);

    // Boucle ROS
    ros::Rate loop_rate(10);  // Fréquence à laquelle traiter les messages (ici 10 Hz)

    while (ros::ok()) {
        ros::spinOnce();  // Traite les callbacks

        loop_rate.sleep();  // Attend jusqu'à la prochaine itération
    }

    return 0;
}
