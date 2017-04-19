#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>
#include <sac_msgs/Encoder.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace shoulder
{
    // constants
    const char *nodeName = "shoulder_motor";
    const char *subscribe = "shoulderMotor";
    const float pi = 3.14159265;
    const float upperLimit = 2.7053;
    const float lowerLimit = -0.6109;
    const int motorNumber = 1;

    // globals
    int currentTickCount = 0;
    bool disconnected = false;

    // publishers
    ros::Publisher simulator;
    ros::ServiceClient position;
    ros::ServiceClient speed;
}

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    if (msg->pos > shoulder::upperLimit || msg->pos < shoulder::lowerLimit)
    {
        ROS_INFO("The value %f sent to the shoulder motor is out of the valid range of %f to %f.", msg->pos, shoulder::lowerLimit, shoulder::upperLimit);
        return;
    }

    ROS_INFO("shoulder motor moving to %f", msg->pos);

    // simulator
    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    shoulder::simulator.publish(simmsg);
    
    if (!shoulder::disconnected)
    {
        // hardware position
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = shoulder::motorNumber;
        int nextTickCount = (int)(msg->pos * 8000 / shoulder::pi);
        posmsg.request.ticks = nextTickCount - shoulder::currentTickCount;

        if (shoulder::position.call(posmsg))
            ROS_INFO("Base motor position written.");
        else
            ROS_INFO("Base motor position failed to write.");

        if (posmsg.response.disconnected)
        {
            shoulder::disconnected = true;
            return;
        }

        // hardware speed
        sac_msgs::MotorSpeed spdmsg;
        spdmsg.request.motor = shoulder::motorNumber;
        spdmsg.request.speed = msg->speed;

        if (shoulder::speed.call(spdmsg))
            ROS_INFO("Base motor speed written.");
        else
            ROS_INFO("Base motor speed failed to write.");

        if (spdmsg.response.disconnected)
        {
            shoulder::disconnected = true;
            return;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, shoulder::nodeName);

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(shoulder::subscribe, 1000, callback);

    shoulder::simulator = nh.advertise<std_msgs::Float64>("scorbot/shoulder_position_controller/command",   1000);
    shoulder::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    shoulder::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::ServiceClient encoder = nh.serviceClient<sac_msgs::Encoder>("encoder");

    // Set the speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = shoulder::motorNumber;
    spdmsg.request.speed = 1;

    shoulder::speed.call(spdmsg);

    if (spdmsg.response.disconnected)
        shoulder::disconnected = true;

    if (!shoulder::disconnected)
    {
        // pos message
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = shoulder::motorNumber;
        posmsg.request.ticks = -5;
        shoulder::position.call(posmsg);

        sac_msgs::Encoder encmsg;
        encmsg.request.motor = shoulder::motorNumber;
        sleep(1);
        
        bool end = false;
        while (!end)
        {
            posmsg.request.ticks = 1;
            shoulder::position.call(posmsg);

            encoder.call(encmsg);
            end = encmsg.response.result;
        }
        posmsg.request.ticks = -6889;
        shoulder::position.call(posmsg);
    }

    ros::spin();

    return 0;
}
