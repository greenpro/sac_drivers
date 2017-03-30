#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#define PI (3.141592)

//ros::Publisher feedback;
ros::Publisher simulator;
ros::Publisher microcontroller;

int currentTickCount = 0;

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std_msgs::Float64 simmsg;
    simmsg.data = msg->data[0];

    simulator.publish(simmsg);
    microcontroller.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alpha_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("alphaMotor", 1000, callback);

    // Outgoing messages
    simulator = nh.advertise<std_msgs::Float64>("andreas_arm/delta_position_controller/command",   1000);
    microcontroller       = nh.advertise<std_msgs::Float64>("microcontroller",           1000);

    ros::spin();

    return 0;
}
