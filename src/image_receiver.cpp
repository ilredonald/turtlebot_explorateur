// image_receiver.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher& pub)
{
    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_receiver");

    ros::NodeHandle nh;

    // Souscrire au topic de la caméra
    image_transport::ImageTransport it(nh);
    
    // Déclare un publisher pour publier l'image reçue
    image_transport::Publisher pub = it.advertise("/image_to_process", 1);

    // Souscrire au topic de la caméra
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, 
        boost::bind(imageCallback, _1, boost::ref(pub)));

        
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

    }
}
