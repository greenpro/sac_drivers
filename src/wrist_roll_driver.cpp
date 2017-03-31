#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>

#define PI (3.141592)

ros::Publisher simulator;

int currentTickCount = 0;

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    ROS_INFO("wrist roll motor moving to %f", msg->pos);
    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    simulator.publish(simmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrist_roll_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("wristRollMotor", 1000, callback);

    // Outgoing messages
    simulator = nh.advertise<std_msgs::Float64>("scorbot/wrist_roll_position_controller/command",   1000);

    ros::spin();

    return 0;
}
