#include <ros/ros.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include "helpers/config.h"
#include "sac_msgs/MotorPosition.h"
#include "sac_msgs/MotorSpeed.h"
#include "sac_msgs/MotorsOffset.h"
#include "sac_msgs/MotorsComplete.h"
#include "sac_msgs/Encoder.h"

namespace usb
{
    // constants
    const char *serialFile = "/dev/ttyUSB0";
    const char *nodeName = "usb_driver";

    // globals
    // The program will not attempt to connect after startup due to complexities though this could be added later
    bool disconnected = false;
}

bool motorSpeed(sac_msgs::MotorSpeed::Request  &req,
                sac_msgs::MotorSpeed::Response &res)
{
    ROS_INFO("----------------------------motor speed");
    res.info = 5;
    if (usb::disconnected)
    {
        res.disconnected = true;
        res.result = false;
        return 0;
    }

    int fd = open(usb::serialFile, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(usb::nodeName, "Unable to open %s", usb::serialFile);
        usb::disconnected = true;
        res.disconnected = true;
        res.result = false;
    	ROS_INFO("----------------------------unable to open");
        return 0;
    }

    fcntl(fd, F_SETFL, 0);

    char *data;
    sprintf(data, "%d V %d", req.motor, req.speed % 10);
    // the % 10 makes the 1 through 9 and 0 work from 1 to 10 instead.

    int wr = write(fd, data, 7);
    // a V b

    close(fd);

    res.disconnected = false;
    res.result = true;
    res.info = 5;

    return 0;
}

bool motorPosition(sac_msgs::MotorPosition::Request  &req,
                   sac_msgs::MotorPosition::Response &res)
{
    if (usb::disconnected)
    {
        res.disconnected = true;
        res.result = false;
        return 0;
    }

    int fd = open(usb::serialFile, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(usb::nodeName, "Unable to open %s", usb::serialFile);
        usb::disconnected = true;
        res.disconnected = true;
        res.result = false;
        return 0;
    }

    fcntl(fd, F_SETFL, 0);

    char sign = req.ticks > -1 ? '+' : '-';

    char *data;
    sprintf(data, "%d M %c %04d \n", req.motor, sign, abs(req.ticks));

    int wr = write(fd, data, 11);
    // a M +- bcde \n

    close(fd);

    res.disconnected = false;
    res.result = true;

    return 0;
}

bool motorsOffset(sac_msgs::MotorsOffset::Request  &req,
                  sac_msgs::MotorsOffset::Response &res)
{
    if (usb::disconnected)
    {
        res.disconnected = true;
        res.result = false;
        return 0;
    }

    int fd = open(usb::serialFile, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(usb::nodeName, "Unable to open %s", usb::serialFile);
        usb::disconnected = true;
        res.disconnected = true;
        res.result = false;
        return 0;
    }

    char *data;
    sprintf(data, "U %03d *", req.offset);

    int wr = write(fd, data, 7);
    // U abc *

    close(fd);

    res.disconnected = false;
    res.result = true;

    return 0;
}

bool motorsComplete(sac_msgs::MotorsComplete::Request  &req,
                    sac_msgs::MotorsComplete::Response &res)
{
    if (usb::disconnected)
    {
        res.disconnected = true;
        return 0;
    }

    int fd = open(usb::serialFile, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(usb::nodeName, "Unable to open %s", usb::serialFile);
        usb::disconnected = true;
        res.disconnected = true;
        return 0;
    }

    int wr = write(fd, "A", 7);
    char *data;
    int rd = read(fd, data, 1);
    // A

    close(fd);

    res.disconnected = false;
    res.result = data[0];

    return 0;
}

bool encoder(sac_msgs::Encoder::Request  &req,
             sac_msgs::Encoder::Response &res)
{
    if (usb::disconnected)
    {
        res.disconnected = true;
        res.result = false;
        return 0;
    }
    
    int fd = open(usb::serialFile, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(usb::nodeName, "Unable to open %s", usb::serialFile);
        usb::disconnected = true;
        res.disconnected = true;
        res.result = false;
        return 0;
    }

    char *data;
    sprintf(data, "%d L", req.motor);

    int wr = write(fd, data, 7);
    int rd = read(fd, data, 1);
    // a L

    close(fd);

    if (data == "1")
        res.result = true;
    else
        res.result = false;
    res.disconnected = false;

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, usb::nodeName);
    ros::NodeHandle nh;

    ros::ServiceServer MotorSpeed            = nh.advertiseService("motorSpeed",     motorSpeed    );
    ros::ServiceServer MotorPosition         = nh.advertiseService("motorPosition",  motorPosition );
    ros::ServiceServer serviceMotorsOffset   = nh.advertiseService("motorsOffset",   motorsOffset  );
    ros::ServiceServer serviceMotorsComplete = nh.advertiseService("motorsComplete", motorsComplete);
    ros::ServiceServer serviceEncoder        = nh.advertiseService("encoder",        encoder       );

    ros::spin();

    return 0;
}
