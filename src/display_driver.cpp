#define RASPI
#define DEBUG

#include <ros/ros.h>
#include <std_msgs/String>
#include <std_msgs/Int32>

#ifdef RASPI
#include <wiringPi.h>
#endif

#define NODE_NAME "display_driver"
#define PREFIX "Display Driver %s"
#define CHILDREN_LEN 10

// Code for the Adafruit LCD plate from http://wiringpi.com/examples/adafruit-rgb-lcd-plate-and-wiringpi/

#define AF_BASE    100
#define AF_R       (AF_BASE + 6 )
#define AF_G       (AF_BASE + 7 )
#define AF_B       (AF_BASE + 8 )

#define AF_E       (AF_BASE + 13)
#define AF_RW      (AF_BASE + 14)
#define AF_RS      (AF_BASE + 15)

#define AF_DB4     (AF_BASE + 12)
#define AF_DB5     (AF_BASE + 11)
#define AF_DB6     (AF_BASE + 10)
#define AF_DB7     (AF_BASE + 9 )

#define AF_ENTER   (AF_BASE + 0 )
#define AF_RIGHT   (AF_BASE + 1 )
#define AF_DOWN    (AF_BASE + 2 )
#define AF_UP      (AF_BASE + 3 )
#define AF_LEFT    (AF_BASE + 4 )

/*
 * about
 *  status
 *  ip address
 *  reboot
 * mode
 *  Towers of Hannoi Controller
 *  Custom Controller
 *  API Controller
 */

struct menuItem
{
    menuItem(string label, int msg, ros::Publisher *pub) : label(label)
    {
        parent = nullptr;
        up = nullptr;
        down = nullptr;
        child = nullptr;
    }
    ~menuItem() {}

    menuItem *parent;
    menuItem *up;
    menuItem *down;
    menuItem *child;

    string label;

    std_msgs::Int32 msg;
    ros::Publisher *pub;
}

// TODO :: Add a callback to display messages on the display with different colors temperarily then display the menu again.

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

#ifdef DEBUG
    ROS_INFO(PREFIX, "node started");
#endif

    ros::NodeHandle nh;
    ros::Publisher modePub = nh.advertise<std_msgs::Float64>("mode", 1000);

    // top level
    menuItem *about = new menuItem("About", NULL, nullptr);
    menuItem *mode = new menuItem("Mode", NULL, nullptr);

    // about menus
    menuItem *status = new menuItem("Status", NULL, nullptr);
    menuItem *ipAddress = new menuItem("IP Address", NULL, nullptr);
    menuItem *reboot = new menuItem("Reboot", NULL, nullptr);

    // mode menus
    menuItem *towersOfHannoi = new menuItem("Towers of Hannoi", 0, modePub);
    menuItem *custom = new menuItem("Custom", 1, modePub);
    menuItem *api new menuItem("API", 2, modePub);

    about->down = mode;
    about->child = status;

    mode->up = node;
    mode->child = towersOfHannoi;

    status->parent = about;
    status->down = ipAddress;

    ipAddress->parent = about;
    ipAddress->up = status;
    ipAddress->down = reboot;

    reboot->parent = about;
    reboot->up = ipAddress;

    towersOfHannoi->parent = mode;
    towersOfHannoi->down = custom;
    
    custom->parent = mode;
    custom->up = towersOfHannoi;
    custom->down = api;

    api->parent = mode;
    api->up = custom;

#ifdef RASPI
    wiringPiSetup();
    mcp23017Setup(AF_BASE, 0x20);
    auto lcd = lcdInit(2, 16, 4 AF_RS, AF_E, AF_DB4, AF_DB5, AF_DB6, AF_DB7, 0, 0, 0, 0);
#endif

#ifdef DEBUG
    ROS_INFO(PREFIX, "Entering loop");
#endif

    while (1)
    {
        //check the buttons
        if (digitalRead(AF_ENTER) == HIGH)
        {
        }
        else if (digitalRead(AF_UP) == HIGH)
        {
        }
        else if (digitalRead(AF_DOWN) == HIGH)
        {
        }
        else if (digitalRead(AF_LEFT) == HIGH)
        {
        }
        else if (digitalRead(AF_RIGHT) == HIGH)
        {
        }
    }

    return 0;
}
