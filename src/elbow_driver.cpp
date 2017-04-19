#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>
#include <sac_msgs/Encoder.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace elbow 
{
    // constants
    const char *nodeName = "elbow_motor";
    const char *subscribe = "elbowMotor";
    const float pi = 3.14159265;
    const float upperLimit = 2.22689;
    const float lowerLimit = -2.22689;
    const int motorNumber = 2;

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
    if (msg->pos > elbow::upperLimit || msg->pos < elbow::lowerLimit)
    {
        ROS_INFO("The value %f sent to the elbow motor is out of the valid range of %f to %f.", msg->pos, elbow::lowerLimit, elbow::upperLimit);
        return;
    }

    // simulator
    ROS_INFO("elbow motor moving to %f", msg->pos);

    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    elbow::simulator.publish(simmsg);
    
    if (!elbow::disconnected)
    {
        // hardware position
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = elbow::motorNumber;
        int nextTickCount = (int)(msg->pos * 8000 / elbow::pi);
        posmsg.request.ticks = nextTickCount - elbow::currentTickCount;

        if (elbow::position.call(posmsg))
            ROS_INFO("elbow motor position written.");
        else
            ROS_INFO("elbow motor position failed to write.");

        if (posmsg.response.disconnected)
        {
            elbow::disconnected = true;
            return;
        }

        // hardware speed
        sac_msgs::MotorSpeed spdmsg;
        spdmsg.request.motor = elbow::motorNumber;
        spdmsg.request.speed = msg->speed;

        if (elbow::speed.call(spdmsg))
            ROS_INFO("elbow motor speed written.");
        else
            ROS_INFO("elbow motor speed failed to write.");

        if (spdmsg.response.disconnected)
        {
            elbow::disconnected = true;
            return;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, elbow::nodeName);

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(elbow::subscribe, 1000, callback);

    // Outgoing messages
    elbow::simulator = nh.advertise<std_msgs::Float64>("scorbot/elbow_position_controller/command",   1000);
    elbow::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    elbow::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::ServiceClient encoder = nh.serviceClient<sac_msgs::Encoder>("encoder");

    // set the speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = elbow::motorNumber;
    spdmsg.request.speed = 1;

    elbow::speed.call(spdmsg);

    if (spdmsg.response.disconnected)
        elbow::disconnected = true;

    if (!elbow::disconnected)
    {
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = elbow::motorNumber;
        posmsg.request.ticks = -5;
        elbow::position.call(posmsg);

        sac_msgs::Encoder encmsg;
        encmsg.request.motor = elbow::motorNumber;
        sleep(1);

        bool end = false;
        while (!end)
        {
            posmsg.request.ticks = 1;
            elbow::position.call(posmsg);

            encoder.call(encmsg);
            end = encmsg.response.result;
        }
        posmsg.request.ticks = -4000;
        elbow::position.call(posmsg);
    }

    ros::spin();

    return 0;
}
