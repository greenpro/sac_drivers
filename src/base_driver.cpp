#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>
#include <sac_msgs/Encoder.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace base 
{
    // constants
    const char *nodeName = "base_motor";
    const char *subscribe = "baseMotor";
    const float pi = 3.14159265;
    const float upperLimit = 2.7053;
    const float lowerLimit = -2.7053;
    const int motorNumber = 0;

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
    if (msg->pos > base::upperLimit || msg->pos < base::lowerLimit)
    {
        ROS_INFO("The value %f sent to the base motor is out of the valid range of %f to %f.", msg->pos, base::upperLimit, base::lowerLimit);
        return;
    }

    ROS_INFO("base motor moving to %f", msg->pos);

    // simulator
    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    base::simulator.publish(simmsg);
    
    if (!base::disconnected)
    {
        // hardware position
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = base::motorNumber;
        int nextTickCount = (int)(msg->pos * 8000 / base::pi);
        posmsg.request.ticks = nextTickCount - base::currentTickCount;

        if (base::position.call(posmsg))
            ROS_INFO("Base motor position written.");
        else
            ROS_INFO("Base motor position failed to write.");

        if (posmsg.response.disconnected)
        {
            base::disconnected = true;
            return;
        }

        // hardware speed
        sac_msgs::MotorSpeed spdmsg;
        spdmsg.request.motor = base::motorNumber;
        spdmsg.request.speed = msg->speed;

        if (base::speed.call(spdmsg))
            ROS_INFO("Base motor speed written.");
        else
            ROS_INFO("Base motor speed failed to write.");

        if(spdmsg.response.disconnected)
        {
            base::disconnected = true;
            return;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, base::nodeName);

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(base::subscribe, 1000, callback);

    // Services
    base::simulator = nh.advertise<std_msgs::Float64>("scorbot/base_position_controller/command",   1000);
    base::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    base::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::ServiceClient encoder = nh.serviceClient<sac_msgs::Encoder>("encoder");

    // Set the speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = base::motorNumber;
    spdmsg.request.speed = 1;

    base::speed.call(spdmsg);

    if (spdmsg.response.disconnected)
        base::disconnected = true;

    if (!base::disconnected)
    {
        // pos message
        sac_msgs::MotorPosition posmsg;
        posmsg.request.motor = base::motorNumber;
        posmsg.request.ticks = -5;
        base::position.call(posmsg);

        sac_msgs::Encoder encmsg;
        encmsg.request.motor = base::motorNumber;
        sleep(1);
    
        bool end = false;
        while (!end)
        {
            posmsg.request.ticks = 1;
            base::position.call(posmsg);

            encoder.call(encmsg);
            end = encmsg.response.result;
        }
        posmsg.request.ticks = -6889;
        base::position.call(posmsg);
    }

    ros::spin();

    return 0;
}
