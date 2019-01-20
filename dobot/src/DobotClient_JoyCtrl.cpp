#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetJOGCmd.h"
#include "dobot/GetPose.h"
#include <cstdlib>
#include <sensor_msgs/Joy.h>

#include<iostream>
//#include<conio.h>

#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>


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
#define KEYCODE_STOP 0x00
ros::Subscriber joy_sub;
sensor_msgs::Joy joydata;



dobot::SetJOGCmd srv_JOG;
ros::ServiceClient client_JOG;
unsigned char c = KEYCODE_STOP, p0 = KEYCODE_STOP, p1 = KEYCODE_STOP, p2 = KEYCODE_STOP, p4 = KEYCODE_STOP;

void joydata2char(unsigned char& c, const sensor_msgs::Joy::ConstPtr& msg)
{
  joydata = *msg;
  //pivot0
  if (joydata.axes[0] > -0.5 && joydata.axes[0] < 0.5) {
    p0 = KEYCODE_STOP;
  }
  else {
    if (joydata.axes[0] >= 0.5) {
      p0 = KEYCODE_S;
    }
    if (joydata.axes[0] <= -0.5) {
      p0 = KEYCODE_W;
    }
  }
  //pivot1
  if (joydata.axes[1] > -0.5 && joydata.axes[1] < 0.5) {
    p1 = KEYCODE_STOP;
  }
  else {
    if (joydata.axes[1] >= 0.5) {
      p1 = KEYCODE_A;
    }
    if (joydata.axes[1] <= -0.5) {
      p1 = KEYCODE_D;
    }
  }


   //pivot3
   if (joydata.axes[4] > -0.5 && joydata.axes[4] < 0.5) {
    p4 = KEYCODE_STOP;
  }
  else {
    if (joydata.axes[4] >= 0.5) {
      p4 = KEYCODE_U;
    }
    if (joydata.axes[4] <= -0.5) {
      p4 = KEYCODE_I;
    }
  }

  if (p0 != KEYCODE_STOP) {
    c = p0;
  } else {
    if (p1 != KEYCODE_STOP) {
      c = p1;
    }
    else {
      if (p4 != KEYCODE_STOP) {
        c = p4;
      }
      else {
        c = KEYCODE_STOP;
      }

    }
  }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr & msg)
{


  joydata = *msg;
  for (int i = 0; i <= 7; i++) {
    std::cout << joydata.axes[i];

  }
  std::cout << std::endl;

  joydata2char(c, msg);



  switch (c) {
  case KEYCODE_W:
    ROS_INFO("W");
    srv_JOG.request.cmd = 1;
    break;
  case KEYCODE_S:
    ROS_INFO("S");
    srv_JOG.request.cmd = 2;
    break;
  case KEYCODE_A:
    ROS_INFO("A");
    srv_JOG.request.cmd = 3;
    break;
  case KEYCODE_D:
    ROS_INFO("D");
    srv_JOG.request.cmd = 4;
    break;
  case KEYCODE_U:
    ROS_INFO("U");
    srv_JOG.request.cmd = 5;
    break;
  case KEYCODE_I:
    ROS_INFO("I");
    srv_JOG.request.cmd = 6;
    break;
  case KEYCODE_J:
    ROS_INFO("J");
    srv_JOG.request.cmd = 7;
    break;
  case KEYCODE_K:
    ROS_INFO("K");
    srv_JOG.request.cmd = 8;
    break;
  default:
    ROS_INFO("DEFAULT:0x%02x", c);
    srv_JOG.request.cmd = 0;
    break;
  }
  if (client_JOG.call(srv_JOG)) {
    ROS_INFO("Result:%d", srv_JOG.response.result);
  } else {
    ROS_ERROR("Failed to call SetJOGCmd");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DobotClient_JoyCtrl");
  ros::NodeHandle n;
  ros::Rate loop_rate(1000);

  client_JOG = n.serviceClient<dobot::SetJOGCmd>("/DobotServer/SetJOGCmd");
  joy_sub = n.subscribe("/joy", 10, joyCallback);

  while (ros::ok()) {
    loop_rate.sleep();


    ros::spinOnce();
  }

  return 0;
}
