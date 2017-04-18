#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>
#include <sac_msgs/Encoder.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace baseMotor 
{
    // constants
    const float pi = 3.14159265;
    const float upperLimit = 2.7053;
    const float lowerLimit = -2.7053;
    const int motorNumber = 0;

    // globals
    int currentTickCount = 0;

    // publishers
    ros::Publisher simulator;
    ros::ServiceClient position;
    ros::ServiceClient speed;
}

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    if (msg->pos > baseMotor::upperLimit || msg->pos < baseMotor::lowerLimit)
    {
        ROS_INFO("The value %f sent to the base motor is out of the valid range of %f to %f.", msg->pos, baseMotor::upperLimit, baseMotor::lowerLimit);
        return;
    }

    ROS_INFO("base motor moving to %f", msg->pos);

    // simulator
    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    baseMotor::simulator.publish(simmsg);
    
    // hardware position
    sac_msgs::MotorPosition posmsg;
    posmsg.request.motor = baseMotor::motorNumber;
    int nextTickCount = (int)(msg->pos * 8000 / baseMotor::pi);
    posmsg.request.ticks = nextTickCount - baseMotor::currentTickCount;

    if (baseMotor::position.call(posmsg))
        ROS_INFO("Base motor position written.");
    else
        ROS_INFO("Base motor position failed to write.");

    // hardware speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = baseMotor::motorNumber;
    spdmsg.request.speed = msg->speed;

    if (baseMotor::speed.call(spdmsg))
        ROS_INFO("Base motor speed written.");
    else
        ROS_INFO("Base motor speed failed to write.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("baseMotor", 1000, callback);

    // Services
    baseMotor::simulator = nh.advertise<std_msgs::Float64>("scorbot/base_position_controller/command",   1000);
    baseMotor::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    baseMotor::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::ServiceClient encoder = nh.serviceClient<sac_msgs::Encoder>("encoder");

    // Set the speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = baseMotor::motorNumber;
    spdmsg.request.speed = 1;

    baseMotor::speed.call(spdmsg);

    // pos message
    sac_msgs::MotorPosition posmsg;
    posmsg.request.motor = baseMotor::motorNumber;
    posmsg.request.ticks = -5;
    baseMotor::position.call(posmsg);

    sac_msgs::Encoder encmsg;
    encmsg.request.motor = baseMotor::motorNumber;
    sleep(1);
    
    bool end = false;
    while (!end)
    {
        posmsg.request.ticks = 1;
        baseMotor::position.call(posmsg);

        encoder.call(encmsg);
        end = encmsg.response.result;
    }
    posmsg.request.ticks = -6889;
    baseMotor::position.call(posmsg);

    ros::spin();

    return 0;
}
