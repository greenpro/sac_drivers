#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define PI (3.141592)
// Fully open
#define UPPER_LIMIT (100)
// Fully closed
#define LOWER_LIMIT (0)

ros::Publisher simulator;
ros::Publisher microcontroller;

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data[0] > UPPER_LIMIT || msg->data[0] < LOWER_LIMIT)
    {
        ROS_INFO("The value %f sent to the delta motor is out of the valid range of %f to %f.", msg->data, UPPER_LIMIT, LOWER_LIMIT);

        return;
    }

    std_msgs::Float64 simmsg;
    simmsg.data = msg->data[0];

    simulator.publish(simmsg);
    microcontroller.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_driver");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("handDriver", 1000, callback);

    // Outgoing messages
    // TODO :: Change this later.
    simulator       = nh.advertise<std_msgs::Float64>("andreas_arm/delta_position_controller/command", 1000);
    microcontroller = nh.advertise<std_msgs::Float64>("microcontroller",                               1000);

    ros::spin();

    return 0;
}
