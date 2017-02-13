#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define PI (3.141592)
#define UPPER_LIMIT (PI)
#define LOWER_LIMIT (-PI)

//ros::Publisher feedback;
ros::Publisher simulator;
ros::Publisher microcontroller;

void callback(const std_msgs::Float64::ConstPtr& msg)
{
    if (msg->data > UPPER_LIMIT || msg->data < LOWER_LIMIT)
    {
        //std_msgs::String msgfb;
        //msgfb.data = "Value is out of range";
        //feedback.publish(msgfb);

        ROS_INFO("The value %f sent to the zeta motor is out of the valid range of %f to %f.", msg->data, UPPER_LIMIT, LOWER_LIMIT);

        return;
    }

    simulator.publish(msg);
    microcontroller.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zetaMotor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("zetaMotor", 1000, callback);

    // Outgoing messages
    //feedback  = nh.advertise<std_msgs::Float64>("zetaFeedback", 1000);
    simulator = nh.advertise<std_msgs::Float64>("andreas_arm/zeta_position_controller/command",   1000);
    microcontroller       = nh.advertise<std_msgs::Float64>("microcontroller",           1000);

    ros::spin();

    return 0;
}
