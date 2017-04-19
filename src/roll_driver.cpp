#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>
#include <sac_msgs/Encoder.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace roll 
{
    // constants
    const char *nodeName = "roll_motor";
    const char *subscribe = "rollMotor";
    const float pi = 3.14159265;
    const int motorNumber = 4;

    // globals
    int currentTickCount = 0;
    bool disconnected = true;

    // publishers
    ros::Publisher simulator;
    ros::ServiceClient position;
    ros::ServiceClient speed;
}

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    ROS_INFO("wrist roll motor moving to %f", msg->pos);
    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    roll::simulator.publish(simmsg);

    if (!roll::disconnected)
    {
        // hardware position
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = roll::motorNumber;
        int nextTickCount = (int)(msg->pos * 8000 / roll::pi);
        posmsg.request.ticks = nextTickCount - roll::currentTickCount;

        if (roll::position.call(posmsg))
            ROS_INFO("roll motor position written.");
        else
            ROS_INFO("roll motor position failed to write.");

        if (posmsg.response.disconnected)
        {
            roll::disconnected = true;
            return;
        }

        // hardware speed
        sac_msgs::MotorSpeed spdmsg;
        spdmsg.request.motor = roll::motorNumber;
        spdmsg.request.speed = msg->speed;

        if (roll::speed.call(spdmsg))
            ROS_INFO("roll motor speed written.");
        else
            ROS_INFO("roll motor speed failed to write.");

        if (spdmsg.response.disconnected)
        {
            roll::disconnected = true;
            return;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, roll::nodeName);

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(roll::subscribe, 1000, callback);

    // Outgoing messages
    roll::simulator = nh.advertise<std_msgs::Float64>("scorbot/wrist_roll_position_controller/command",   1000);
    roll::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    roll::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::ServiceClient encoder = nh.serviceClient<sac_msgs::Encoder>("encoder");

    // Set the speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = roll::motorNumber;
    spdmsg.request.speed = 1;

    roll::speed.call(spdmsg);

    if (spdmsg.response.disconnected)
        roll::disconnected = true;

    if (!roll::disconnected)
    {
        // pos message
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = roll::motorNumber;
        posmsg.request.ticks = -5;
        roll::position.call(posmsg);

        sac_msgs::Encoder encmsg;
        encmsg.request.motor = roll::motorNumber;
        sleep(1);
    
        bool end = false;
        while (!end)
        {
            posmsg.request.ticks = 1;
            roll::position.call(posmsg);

            encoder.call(encmsg);
            end = encmsg.response.result;
        }
        posmsg.request.ticks = -4000;
        roll::position.call(posmsg);
    }

    ros::spin();

    return 0;
}
