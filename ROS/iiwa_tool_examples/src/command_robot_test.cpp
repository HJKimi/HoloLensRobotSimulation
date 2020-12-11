#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <math.h>

//Socket comm
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

//read reachable planefiles
#include <iostream>
#include <fstream>

//Socket comm
#define BUF_SIZE 28
#define portnum 9195

//#define ros_rate 10.0
#define threshold 0.2

using namespace std;

iiwa_msgs::JointPosition current_joint_position, command_joint_position;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
std::string joint_position_topic, cartesian_position_topic, command_cartesian_position_topic, command_joint_position_topic;

bool isRobotConnected = false, use_cartesian_command = true;

double x, y, z; //hand position from openni_tracker node
double forward_headx, turn_heady, headz; //avatar vel/rot from openni_tracker node
int init;
double a, b, dist;
bool modechange = false, movemode = true;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp)
{
	if (!isRobotConnected)
	isRobotConnected = !isRobotConnected;
	current_joint_position = jp;
}

void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps)
{
	if (!isRobotConnected)
	isRobotConnected = !isRobotConnected;
	current_cartesian_position = ps;
}

int main (int argc, char **argv)
{

	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Create Transform listener object
	tf::TransformListener listener;

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	nh.param("joint_position_topic", joint_position_topic, std::string("/iiwa/state/JointPosition"));
	nh.param("cartesian_position_topic", cartesian_position_topic, std::string("/iiwa/state/CartesianPose"));
	nh.param("command_cartesian_position_topic", command_cartesian_position_topic, std::string("/iiwa/command/CartesianPose"));
	nh.param("command_joint_position_topic", command_joint_position_topic, std::string("/iiwa/command/JointPosition"));
	nh.param("use_cartesian_command", use_cartesian_command, true);

	// Subscribers and publishers

	ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
	ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
	ros::Publisher pub_joint_command = nh.advertise<iiwa_msgs::JointPosition>(command_joint_position_topic, 1);

	ros::Publisher pub_cartesian_command = nh.advertise<geometry_msgs::PoseStamped>(command_cartesian_position_topic, 1);


	while (ros::ok())
	{
		if (isRobotConnected)
		{
			command_cartesian_position = current_cartesian_position;
			command_cartesian_position.pose.position.x += 0.005;
			command_cartesian_position.pose.orientation.x = 0.2706;
			command_cartesian_position.pose.orientation.y = 0.6533;
			command_cartesian_position.pose.orientation.z = -0.2706;
			command_cartesian_position.pose.orientation.w = 0.6533;

			pub_cartesian_command.publish(command_cartesian_position);
		}



		else {
			ROS_ERROR("Robot is not connected...");
			ros::Duration(2.0).sleep(); // 5 seconds
		}
	}

	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;

};
