#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat latest_frame;
sensor_msgs::ImageConstPtr latest_msg;
bool new_image_received = false;

ros::Publisher image_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        // Convertir l'image en niveaux de gris
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        // Appliquer CLAHE
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(10.0, cv::Size(8, 8));
        cv::Mat enhanced;
        clahe->apply(gray, enhanced);

        // Convertir en BGR pour republication
        cv::cvtColor(enhanced, latest_frame, cv::COLOR_GRAY2BGR);
        latest_msg = msg;
        new_image_received = true;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Erreur cv_bridge : %s", e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "traitement_image");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/camera/image", 1, imageCallback);
    image_pub = nh.advertise<sensor_msgs::Image>("/camera/image_processed", 1);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (new_image_received) {
            sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(
                latest_msg->header, "bgr8", latest_frame).toImageMsg();
            image_pub.publish(output_msg);

            ROS_INFO("Image traitée publiée.");
            new_image_received = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
