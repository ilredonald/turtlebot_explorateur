#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0);
cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);  // Supposons que l'objectif est parfait

class Object3DIntegration
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // Points 3D de l'objet que nous voulons afficher (exemple : une boîte)
    std::vector<cv::Point3f> objectPoints_;

public:
    Object3DIntegration(ros::NodeHandle& nh) : it_(nh)
    {
        // Souscription au flux vidéo traité du robot (image traitée par traitement_image)
        image_sub_ = it_.subscribe("/camera/image_processed", 1, &Object3DIntegration::imageCallback, this);
        image_pub_ = it_.advertise("/image_with_3d_object", 1);

        // Définition des points de l'objet 3D (ex. une boîte)
        objectPoints_.push_back(cv::Point3f(-0.5, -0.5, 1.0));  // Coin inférieur gauche
        objectPoints_.push_back(cv::Point3f(0.5, -0.5, 1.0));   // Coin inférieur droit
        objectPoints_.push_back(cv::Point3f(0.5, 0.5, 1.0));    // Coin supérieur droit
        objectPoints_.push_back(cv::Point3f(-0.5, 0.5, 1.0));   // Coin supérieur gauche
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            // Conversion de l'image ROS en OpenCV
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Définition d'une position arbitraire pour l'objet 3D dans l'espace
            cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);  // Rotation de 0 sur les 3 axes
            cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 2.0);  // Position de l'objet en face de la caméra

            // Projection de l'objet 3D sur l'image 2D
            std::vector<cv::Point2f> imagePoints;
            cv::projectPoints(objectPoints_, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

            // Dessiner l'objet 3D projeté sur l'image (ici une boîte)
            for (size_t i = 0; i < 4; i++)
            {
                cv::line(frame, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
            }

            // Optionnel : dessiner une ligne diagonale pour mieux visualiser l'objet
            cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 0, 255), 2);  // Diagonale rouge
            cv::line(frame, imagePoints[1], imagePoints[3], cv::Scalar(0, 0, 255), 2);  // Diagonale rouge

            // Publier l'image avec l'objet 3D intégré
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            image_pub_.publish(out_msg);

        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Erreur de conversion d'image : %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_3d_integration");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Object3DIntegration object3D(nh);

    while (ros::ok())
    {
        ROS_INFO("Object3DIntegration node is running...");
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
}
