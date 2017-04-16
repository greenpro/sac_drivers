#include <ros/ros.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include "helpers/config.h"
#include "sac_msgs/motorPosition.h"
#include "sac_msgs/motorSpeed.h"
#include "sac_msgs/motorsOffset.h"
#include "sac_msgs/motorsComplete.h"
#include "sac_msgs/encoder.h"

#define SERIAL_FILE ("/dev/ttyusb0")
#define NODE_NAME ("usb_driver")

bool motorSpeed(sac_msgs::motorSpeed::Request  &req,
                sac_msgs::motorSpeed::Response &res)
{
    int fd;
    char *data;
    int wr;
    int rd;

    fd = open(SERIAL_FILE, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(NODE_NAME, "Unable to open %s", SERIAL_FILE);
        return 1;
    }

    fcntl(fd, F_SETFL, 0);

    ROS_INFO_NAMED(NODE_NAME, "File open %d", fd);

    sprintf(data, "%d V %d", req.motor, req.speed % 10);
    // the % 10 makes the 1 through 9 and 0 work from 1 to 10 instead.

    wr = write(fd, data, 7);
    // a V b

    res.result = true;

    return 0;
}

bool motorPosition(sac_msgs::motorPosition::Request  &req,
                   sac_msgs::motorPosition::Response &res)
{
    int fd;
    char *data;
    int wr;
    int rd;

    fd = open(SERIAL_FILE, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(NODE_NAME, "Unable to open %s", SERIAL_FILE);
        return 1;
    }

    fcntl(fd, F_SETFL, 0);

    ROS_INFO_NAMED(NODE_NAME, "File open %d", fd);

    char sign = '+';
    if (req.angle < 0)
        sign = '-';

    sprintf(data, "%d M %c %04d \n", req.motor, sign, abs(req.angle));

    wr = write(fd, data, 11);
    // a M +- bcde \n

    res.result = true;

    return 0;
}

bool motorsOffset(sac_msgs::motorsOffset::Request  &req,
                  sac_msgs::motorsOffset::Response &res)
{
    int fd;
    char *data;
    int wr;
    int rd;

    fd = open(SERIAL_FILE, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(NODE_NAME, "Unable to open %s", SERIAL_FILE);
        return 1;
    }

    sprintf(data, "U %03d *", req.offset);

    wr = write(fd, data, 7);
    // U abc *

    res.result = true;

    return 0;
}

bool motorsComplete(sac_msgs::motorsComplete::Request  &req,
                    sac_msgs::motorsComplete::Response &res)
{
    int fd;
    char *data;
    int wr;
    int rd;

    fd = open(SERIAL_FILE, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(NODE_NAME, "Unable to open %s", SERIAL_FILE);
        return 1;
    }

    wr = write(fd, "A", 7);
    rd = read(fd, data, 1);
    // A

    res.result = data[0];

    return 0;
}

bool encoder(sac_msgs::encoder::Request  &req,
             sac_msgs::encoder::Response &res)
{
    int fd;
    char *data;
    int wr;
    int rd;

    fd = open(SERIAL_FILE, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        ROS_INFO_NAMED(NODE_NAME, "Unable to open %s", SERIAL_FILE);
        return 1;
    }

    sprintf(data, "%d L", req.motor);

    wr = write(fd, data, 7);
    rd = read(fd, data, 1);
    // a L

    res.result = data[0];

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_server");
    ros::NodeHandle nh;

    ros::ServiceServer MotorSpeed            = nh.advertiseService("motorSpeed",     motorSpeed    );
    ros::ServiceServer MotorPosition         = nh.advertiseService("motorPosition",  motorPosition );
    ros::ServiceServer serviceMotorsOffset   = nh.advertiseService("motorsOffset",   motorsOffset  );
    ros::ServiceServer serviceMotorsComplete = nh.advertiseService("motorsComplete", motorsComplete);
    ros::ServiceServer serviceEncoder        = nh.advertiseService("encoder",        encoder       );

    ros::spin();

    return 0;
}
