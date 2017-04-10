#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/HandPos.h>

#define PI (3.141592)
#define UPPER_LIMIT (0.065)
#define LOWER_LIMIT (0)

ros::Publisher gripper0;
ros::Publisher gripper1;

int currentTickCount = 0;

void callback(const sac_msgs::HandPos::ConstPtr& msg)
{
    if (msg->width > UPPER_LIMIT || msg->width < LOWER_LIMIT)
    {
        ROS_INFO("The value %f sent to the hand motor is out of the valid range of %f to %f.", msg->width, UPPER_LIMIT, LOWER_LIMIT);
        return;
    }

    ROS_INFO("Grippers moving to %f", msg->width);

    float w = msg->width;
    float w_max = 0.065;
    float Hc = 1.087968364;
    float g = 0.070;

    float H = acos((w_max - w) / (2 * g)) - Hc;

    std_msgs::Float64 g0;
    std_msgs::Float64 g1;

    ROS_INFO("Grippers moving to angle %f", H);

    g0.data = H;
    g1.data = H;

    gripper0.publish(g0);
    gripper1.publish(g1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_driver");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("handDriver", 1000, callback);

    // Outgoing messages
    gripper0 = nh.advertise<std_msgs::Float64>("scorbot/pad1_position_controller/command",   1000);
    gripper1 = nh.advertise<std_msgs::Float64>("scorbot/pad2_position_controller/command",   1000);

    ros::spin();

    return 0;
}
