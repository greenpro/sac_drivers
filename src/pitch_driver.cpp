#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>
#include <sac_msgs/Encoder.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace pitch 
{
    // constants
    const char *nodeName = "pitch_motor";
    const char *subscribe = "pitchMotor";
    const float pi = 3.14159265;
    const float upperLimit = 2.22689;
    const float lowerLimit = -2.22689;
    const int motorNumber = 3;

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
    if (msg->pos > pitch::upperLimit || msg->pos < pitch::lowerLimit)
    {
        ROS_INFO("The value %f sent to the wrist pitch motor is out of the valid range of %f to %f.", msg->pos, pitch::lowerLimit, pitch::upperLimit);

        return;
    }

    ROS_INFO("wrist pitch motor moving to %f", msg->pos);

    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    pitch::simulator.publish(simmsg);
    
    if (!pitch::disconnected)
    {
        // hardware position
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = pitch::motorNumber;
        int nextTickCount = (int)(msg->pos * 8000 / pitch::pi);
        posmsg.request.ticks = nextTickCount - pitch::currentTickCount;

        if (pitch::position.call(posmsg))
            ROS_INFO("pitch motor position written.");
        else
            ROS_INFO("pitch motor position failed to write.");

        if (posmsg.response.disconnected)
        {
            pitch::disconnected = true;
            return;
        }

        // hardware speed
        sac_msgs::MotorSpeed spdmsg;
        spdmsg.request.motor = pitch::motorNumber;
        spdmsg.request.speed = msg->speed;

        if (pitch::speed.call(spdmsg))
            ROS_INFO("pitch motor speed written.");
        else
            ROS_INFO("pitch motor speed failed to write.");

        if (spdmsg.response.disconnected)
        {
            pitch::disconnected = true;
            return;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, pitch::nodeName);

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(pitch::subscribe, 1000, callback);

    // Outgoing messages
    pitch::simulator = nh.advertise<std_msgs::Float64>("scorbot/wrist_pitch_position_controller/command",   1000);
    pitch::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    pitch::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");
    
    ros::ServiceClient encoder = nh.serviceClient<sac_msgs::Encoder>("encoder");

    // Set the speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = pitch::motorNumber;
    spdmsg.request.speed = 1;

    pitch::speed.call(spdmsg);

    ROS_INFO("-----------------------------------------------------called");

    if (spdmsg.response.disconnected)
        pitch::disconnected = true;

    ROS_INFO("-----------------------------------------------------connected %d", spdmsg.response.info);

    if (!pitch::disconnected)
    {
	ROS_INFO("--------------------------------------------------hardware");
        // pos message
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = pitch::motorNumber;
        posmsg.request.ticks = -5;
        pitch::position.call(posmsg);

        sac_msgs::Encoder encmsg;
        encmsg.request.motor = pitch::motorNumber;
        sleep(1);
    
        bool end = false;
        while (!end)
        {
            posmsg.request.ticks = 1;
            pitch::position.call(posmsg);

            encoder.call(encmsg);
            end = encmsg.response.result;
        }
        posmsg.request.ticks = -6889;
        pitch::position.call(posmsg);
    }

    ros::spin();

    return 0;
}
