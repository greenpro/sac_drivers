#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& msg)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serialDriver");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("serial", 1000, callback);

    ros::spin();

    return 0;
}
