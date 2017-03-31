#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>

#define PI (3.141592)
#define UPPER_LIMIT (2.22689)
#define LOWER_LIMIT (-2.22689)

ros::Publisher simulator;

int currentTickCount = 0;

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    if (msg->pos > UPPER_LIMIT || msg->pos < LOWER_LIMIT)
    {
        ROS_INFO("The value %f sent to the wrist pitch motor is out of the valid range of %f to %f.", msg->pos, UPPER_LIMIT, LOWER_LIMIT);

        return;
    }

    ROS_INFO("wrist pitch motor moving to %f", msg->pos);

    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    simulator.publish(simmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrist_pitch_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("wristPitchMotor", 1000, callback);

    // Outgoing messages
    simulator = nh.advertise<std_msgs::Float64>("scorbot/wrist_pitch_position_controller/command",   1000);

    ros::spin();

    return 0;
}
