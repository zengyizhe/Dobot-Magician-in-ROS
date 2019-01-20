#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "DobotDll.h"

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <math.h>

#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"


void callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr  &msg);
ros::ServiceClient client_1;
ros::ServiceClient client_2;
ros::ServiceClient client_3;
ros::ServiceClient client_4;
ros::ServiceClient client_5;
ros::ServiceClient client_6;
ros::ServiceClient client_7;
ros::ServiceClient client_8;
ros::ServiceClient client_9;
ros::ServiceClient client_10;


int main(int argc, char **argv)
{

	ros::init(argc, argv, "JointTrajectory_subscriber");
	ros::NodeHandle n;
	ros::ServiceClient client;

	client_1 = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");

	client_2 = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");

	client_3 = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");

	client_4 = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");

	client_5 = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");

	client_6 = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");


	client_7 = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");


	client_8 = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");


	client_9 = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");

	client_10 = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");



	ROS_INFO("JointTrajectory_subscriber running.../n");

	ros::Rate loop_rate(30);
	ros::Subscriber trajectory_sub = n.subscribe("/arm_controller/follow_joint_trajectory/goal", 10, callback);

	ros::spin();


	return 0;
}

void callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr  &msg)
{

	int trajectory_length = msg->goal.trajectory.points.size();
	std::cout << "/n/nlength of trajectory points is:" << trajectory_length << std::endl << std::endl;


	// SetCmdTimeout
	dobot::SetCmdTimeout srv1;
	srv1.request.timeout = 3000;
	if (client_1.call(srv1) == false) {
		ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
		return ;
	}

	// Clear the command queue
	dobot::SetQueuedCmdClear srv2;
	client_2.call(srv2);


	// Start running the command queue
	dobot::SetQueuedCmdStartExec srv3;
	client_3.call(srv3);

	// Get device version information
	dobot::GetDeviceVersion srv4;
	client_4.call(srv4);
	if (srv4.response.result == 0) {
		ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
	} else {
		ROS_ERROR("Failed to get device version information!");
	}

	// Set end effector parameters
	dobot::SetEndEffectorParams srv5;
	srv5.request.xBias = 0;
	srv5.request.yBias = 0;
	srv5.request.zBias = 0;
	client_5.call(srv5);


	// Set PTP joint parameters
	do {
		dobot::SetPTPJointParams srv;

		for (int i = 0; i < 4; i++) {
			srv.request.velocity.push_back(100);
		}
		for (int i = 0; i < 4; i++) {
			srv.request.acceleration.push_back(100);
		}
		client_6.call(srv);
	} while (0);

	// Set PTP coordinate parameters
	do {
		dobot::SetPTPCoordinateParams srv;

		srv.request.xyzVelocity = 100;
		srv.request.xyzAcceleration = 100;
		srv.request.rVelocity = 100;
		srv.request.rAcceleration = 100;
		client_7.call(srv);
	} while (0);

	// Set PTP jump parameters
	do {
		dobot::SetPTPJumpParams srv;

		srv.request.jumpHeight = 20;
		srv.request.zLimit = 200;
		client_8.call(srv);
	} while (0);

	// Set PTP common parameters
	do {
		dobot::SetPTPCommonParams srv;

		srv.request.velocityRatio = 50;

		srv.request.accelerationRatio = 50;
		client_9.call(srv);
	} while (0);

	dobot::SetPTPCmd srv;
/*	for (int i = 0; i < trajectory_length - 1; i++)
	{
		srv.request.ptpMode = 4;
		srv.request.x = (float)msg->goal.trajectory.points[i].positions[0] / (float)M_PI * 180;
		srv.request.y = (float)msg->goal.trajectory.points[i].positions[1] / (float)M_PI * 180;;
		srv.request.z = (float)msg->goal.trajectory.points[i].positions[2] / (float)M_PI * 180;;
		srv.request.r = 0;
		client_10.call(srv);
	}
*/
		/*srv.request.ptpMode = 4;
		srv.request.x = (float)msg->goal.trajectory.points[0].positions[0] / (float)M_PI * 180;
		srv.request.y = (float)msg->goal.trajectory.points[0].positions[1] / (float)M_PI * 180;;
		srv.request.z = (float)msg->goal.trajectory.points[0].positions[2] / (float)M_PI * 180;;
		srv.request.r = 0;
		client_10.call(srv);
*/
		srv.request.ptpMode = 4;
		srv.request.x = (double)msg->goal.trajectory.points[trajectory_length-1].positions[0] / (double)M_PI * 180;
		srv.request.y = (double)msg->goal.trajectory.points[trajectory_length-1].positions[1] / (double)M_PI * 180;;
		srv.request.z = (double)msg->goal.trajectory.points[trajectory_length-1].positions[2] / (double)M_PI * 180;;
		srv.request.r = 0;
		client_10.call(srv);

}