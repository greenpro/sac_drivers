#include <ros/ros.h>
#include <std_msgs/String>
#include <std_msgs/Int32>
#include <linux/reboot.h>
#include <helpers/config.h>
#include <helpers/display_driver.h>
#include <helpers/menuItem.h>

#ifdef RASPBERRY_PI
#include <wiringPi.h>
#include <lcd.h>
#endif

#define NODE_NAME "display_driver"
#define PREFIX "Display Driver %s"
#define CHILDREN_LEN 10

// Code for the Adafruit LCD plate from http://wiringpi.com/examples/adafruit-rgb-lcd-plate-and-wiringpi/

#define AF_BASE    100

display disp;
ros::Publisher modePub;

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

// TODO :: Add a callback to display messages on the display with different colors temperarily then display the menu again.

// Code from: http://www.unix.com/programming/146092-how-do-i-check-if-process-running-c.html
void statusFunc()
{
    bool allRunning = true;
    string out = system("rosnode list");

    // Controllers
    if (std::string::npos != out.find("api_controller"))
    {
        disp.displayText("API Controller\nnot running");
        delay(2);
        allRunning = false;
    }

    if (std::string::npos != out.find("towers_of_hannoi_controller"))
    {
        disp.displayText("Towers of Hannoi\nnot running");
        delay(2);
        allRunning = false;
    }
    
    if (std::string::npos != out.find("custom_controller"))
    {
        disp.displayText("Custom Controller\nnot running");
        delay(2);
        allRunning = false;
    }

    // Translators
    // TODO :: look at merging these two files by using the ifdef statements
#ifdef ANDREAS_ARM
    if (std::string::npos != out.find("andreas_arm_moveit"))
    {
        disp.displayText("Andreas MoveIt\n not running");
        delay(2);
        allRunning = false;
    }
#endif

#ifdef SCORBOT
    if (std::string::npos != out.find("scorbot_moveit"))
    {
        disp.displayText("Scorbot MoveIt\nnot running");
        delay(2);
        allRunning = false;
    }
#endif

    // Drivers
    if (std::string::npos != out.find("alpha_driver"))
    {
        disp.displayText("Alpha Driver\nnot running");
        delay(2);
        allRunning = false;
    }

    if (std::string::npos != out.find("beta_driver"))
    {
        disp.displayText("Beta Driver\nnot running");
        delay(2);
        allRunning = false;
    }

    if (std::string::npos != out.find("gamma_driver"))
    {
        disp.displayText("Gamma Driver\nnot running");
        delay(2);
        allRunning = false;
    }

    if (std::string::npos != out.find("delta_driver"))
    {
        disp.displayText("Delta Driver\nnot running");
        delay(2);
        allRunning = false;
    }

    if (std::string::npos != out.find("epsilon_driver"))
    {
        disp.displayText("Epsilon Driver\nnot running");
        delay(2);
        allRunning = false;
    }
    
    if (std::string::npos != out.find("zeta_driver"))
    {
        disp.displayText("Zeta Driver\nnot running");
        delay(2);
        allRunning = false;
    }

    if (std::string::npos != out.find("serial_driver"))
    {
        disp.displayText("Serial Driver\nnot running");
        delay(2);
        allRunning = false;
    }

    if (!allRunning)
    {
        disp.displayText("All Programs\nRunning", 3);
    }

    // the display driver is already running no need to test.
}

// display the ip address on the screen
void ipAddressFunc()
{
    system("ipconfig > ip.txt");

    ifstream ifile;
    ifile.open("ip.txt");

    string wlanLine;
    string ethLine;
    string line;
    string wlanIp;
    string ethIp;
    
    // Get the Ethernet and WiFi IP addresses
    if (ifile.is_open())
    {
        while (!ifile.eof())
        {
            getline(ifile, line);

            if (line[0] == 'w')
            {
                wlanLine = line;

                if (ethLine)
                    break;
            }

            if (line[0] == 'e')
            {
                ethLine = line;

                if (wlanLine)
                    break;
            }
        }
    }

    // Extract the Ethernet IP address
    bool take = false;
    for (int i=0; ethLine[i]; i++)
    {
        if ('1' <= ethLine[i] && ethLine[i] <= '9' && !take)
        {
            take = true;
        }

        if (take)
        {
            ethIp.append(ethLine[i]);
        }

        if (take && && ethLine[i] && ethLine[i+1] == ' ')
        {
            break;
        }
    }

    // Extract the WiFi IP address
    bool take = false;
    for (int i=0; wlanLine[i]; i++)
    {
        if ('1' <= wlanLine[i] && wlanLine[i] <= '9' && !take)
        {
            take = true;
        }

        if (take)
        {
            wlanIp.append(wlanLine[i]);
        }

        if (take && && wlanLine[i] && wlanLine[i+1] == ' ')
        {
            break;
        }
    }

    // Display to the screen
    line = "Eth: " + ethline + "\nwlan: " + wlanLine
    disp.displayText(ethLine, 30);
}

// reboot the raspberry pi and run a reboot on the robot display the sequence to the screen
void rebootFunc()
{
    disp.displayText("Rebooting System");

    reboot(LINUX_REBOOT_MAGIC1, LINUX_REBOOT_MAGIC2, LINUX_REBOOT_CMD_POWER_OFF, 0);
}

// tell the controllers to run the towers of hannoi controller
void towersOfHannoiFunc()
{
    std_msgs::Float64 msg;
    msg->data = 0;
    modePub.publish(msg);
}

// tell the controllers to run the custom controller
void customFunc()
{
    std_msgs::Float64 msg;
    msg->data = 1;
    modePub.publish(msg);
}

// tell the controllers to run the api controller
void apiFunc()
{
    std_msgs::Float64 msg;
    msg->data = 2;
    modePub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

#ifdef DEBUG
    ROS_INFO(PREFIX, "node started");
#endif

    ros::NodeHandle nh;
    modePub = nh.advertise<std_msgs::Float64>("mode", 1000);

    // top level
    menuItem *about = new menuItem("About", nullptr);
    menuItem *mode = new menuItem("Mode", nullptr);

    // about menus
    menuItem *status = new menuItem("Status", statusFunc);
    menuItem *ipAddress = new menuItem("IP Address", ipAddressFunc);
    menuItem *reboot = new menuItem("Reboot", rebootFunc);

    // mode menus
    menuItem *towersOfHannoi = new menuItem("Towers of Hannoi", towersOfHannoiFunc);
    menuItem *custom = new menuItem("Custom", customFunc);
    menuItem *api new menuItem("API", apiFunc);

    // back level 1
    menuItem *back0 = new menuItem("Back", nullptr);
    menuItem *back1 = new menuItem("Back", nullptr);

    // setup the menu connections
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
    reboot->down = back0;

    back0->parent = about;
    back0->up = reboot;
    back0->child = about;

    towersOfHannoi->parent = mode;
    towersOfHannoi->down = custom;
    
    custom->parent = mode;
    custom->up = towersOfHannoi;
    custom->down = api;

    api->parent = mode;
    api->up = custom;
    api->down = back1;

    back1->parent = mode;
    back1->up = api;
    back1->child = mode;

    // Menu Current Position Pointer
    menuItem *menuPtr = about;

    // setup the display
    disp = display(2, 16, 4, 11, 10, 
            0, 1, 2, 3,
            0, 0, 0, 0,
            AF_BASE);

#ifdef DEBUG
    ROS_INFO(PREFIX, "Entering loop");
#endif

    bool toggle = false;
    while (1)
    {
        //check the buttons
        if (disp.pollSelect() && !toggle)
        {
            if (menuPtr->child)
                pointer = menuPtr->child;
            else
                pointer->func();

            toggle = true;
        }
        else if (disp.pollUp() && !toggle)
        {
            menuPtr = menuPtr->up;
            toggle = true;
        }
        else if (disp.pollDown() && !toggle)
        {
            menuPtr = menuPtr->down;
            toggle = true;
        }
        else if (disp.pollRight() && !toggle)
        {
            if (menuPtr->child)
                pointer = menuPtr->child;
            else
                pointer->func();

            toggle = true;
        }
        else if (disp.pollLeft() && !toggle)
        {
            menuPtr = menuPtr->parent;
            toggle = true;
        }
        else
        {
            toggle = false;
        }
    }

    return 0;
}
