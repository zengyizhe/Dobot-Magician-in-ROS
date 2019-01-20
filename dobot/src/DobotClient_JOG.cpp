#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetJOGCmd.h"
#include <cstdlib>

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b

int kfd = 0;
struct termios cooked, raw;

void keyboardLoop(ros::NodeHandle &n)
{
    unsigned char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("poll():");
            return;
        } else if(num > 0) {
            if(read(kfd, &c, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            continue;
        }
	    // SetJOGCmdService
        ros::ServiceClient client;
        client = n.serviceClient<dobot::SetJOGCmd>("/DobotServer/SetJOGCmd");
        dobot::SetJOGCmd srv;
        srv.request.isJoint = false;
        switch(c) {
            case KEYCODE_W:
                ROS_INFO("W");
                srv.request.cmd = 1;
            break;
            case KEYCODE_S:
                ROS_INFO("S");
                srv.request.cmd = 2;
            break;
            case KEYCODE_A:
                ROS_INFO("A");
                srv.request.cmd = 3;
            break;
            case KEYCODE_D:
                ROS_INFO("D");
                srv.request.cmd = 4;
            break;
            case KEYCODE_U:
                ROS_INFO("U");
                srv.request.cmd = 5;
            break;
            case KEYCODE_I:
                ROS_INFO("I");
                srv.request.cmd = 6;
            break;
            case KEYCODE_J:
                ROS_INFO("J");
                srv.request.cmd = 7;
            break;
            case KEYCODE_K:
                ROS_INFO("K");
                srv.request.cmd = 8;
            break;
            default:
                ROS_INFO("DEFAULT:0x%02x", c);
                srv.request.cmd = 0;
            break;
        }
        if (client.call(srv)) {
	        ROS_INFO("Result:%d", srv.response.result);
        } else {
            ROS_ERROR("Failed to call SetJOGCmd");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    // SetCmdTimeout
    ros::ServiceClient client;

    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv;
    srv.request.timeout = 3000;
    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    boost::thread t = boost::thread(boost::bind(&keyboardLoop, boost::ref(n)));
    ros::spin();
    t.interrupt();
    t.join();

    tcsetattr(kfd, TCSANOW, &cooked);
    return 0;
}

