#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetJOGCmd.h"
#include "dobot/GetPose.h"
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

#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fstream>
#include <string>

int i = 0;
int c = '0';

std::string fileName = "armTrack.csv";

int kbhit()
{
  int i;
  ioctl(0, FIONREAD, &i);
  return i;
}

long getCurrentTime()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}


int key;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "DobotClient_GetPose");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::ServiceClient client_pose = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
  dobot::GetPose srv_pose;

  std::ofstream file(fileName.c_str());
  if (!file) {std::cout << "open file erro" << std::endl; return 0;}

/*  system("stty raw -echo");
  std::cout << "press ctrl+C to out" << std::endl;
  //system("stty raw -echo");*/
  while (ros::ok())
  {
    loop_rate.sleep();

    if (client_pose.call(srv_pose)) {
/*      if (kbhit())
      {
        c = getchar();
        // std::cout<<"the output is"<<c<<std::endl;
        if (c == 'q') { break;}
      }*/

      file << getCurrentTime() << ",";
      file << srv_pose.response.x << "," << srv_pose.response.y << "," << srv_pose.response.z << ",";
      file << c << std::endl;
      //ROS_INFO("x:&f y:&f z:&f r:&f /n",srv_pose.response.x,srv_pose.response.y,srv_pose.response.z,srv_pose.response.r);
       std::cout<<getCurrentTime()<<" ";
       std::cout<<srv_pose.response.x<<" "<<srv_pose.response.y<<" "<<srv_pose.response.z<<" ";
      std::cout<<c<<std::endl;

    }
    ros::spinOnce();
  }
  file.close();
/*  system("stty cooked echo");*/
  return 0;
}
