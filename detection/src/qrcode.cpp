#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <std_msgs/String.h>

class QRDetector
{
public:
    QRDetector(ros::NodeHandle& nh) : it_(nh)
    {
        // S'abonner à /camera/image_processed (image traitée par le noeud traitement_image)
        image_sub_ = it_.subscribe("/camera/image_processed", 1, &QRDetector::imageCallback, this);
        qr_status_pub_ = nh.advertise<std_msgs::String>("/qr_status", 10);

        ROS_INFO("QRDetector node started. Subscribing to: /camera/image_processed");

        // Créer une fenêtre OpenCV pour affichage
        cv::namedWindow("QR Code Detection", cv::WINDOW_NORMAL);
    }

    ~QRDetector()
    {
        cv::destroyWindow("QR Code Detection");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO_ONCE("Receiving processed images from /camera/image_processed...");
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Convertir en niveaux de gris
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        // Détection de QR Code avec ZBar
        zbar::ImageScanner scanner;
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        zbar::Image image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

        int n = scanner.scan(image);
        std_msgs::String qr_status_msg;

        if (n > 0) {
            qr_status_msg.data = "QR code detected";
            for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
            {
                ROS_INFO("QR Code detected: %s", symbol->get_data().c_str());

                std::vector<cv::Point> points;
                for (int i = 0; i < symbol->get_location_size(); i++) {
                    points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
                }
                cv::polylines(cv_ptr->image, points, true, cv::Scalar(0, 255, 0), 2);
            }
        } else {
            qr_status_msg.data = "No QR code detected";
        }

        qr_status_pub_.publish(qr_status_msg);
        cv::imshow("QR Code Detection", cv_ptr->image);
        cv::waitKey(1);
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher qr_status_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_turtlebot_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    QRDetector qr_detector(nh); 

    while (ros::ok())
    {
        ROS_INFO("QRDetector node is running...");
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
}
