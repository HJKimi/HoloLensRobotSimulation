#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"

#define BUF_SIZE 100

void error_handling(char * message);
void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps);
void deduplication();

iiwa_msgs::JointPosition current_joint_position, command_joint_position;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
std::string joint_position_topic, cartesian_position_topic, command_cartesian_position_topic, command_joint_position_topic;
double ros_rate = 0.1;
bool isRobotConnected = false, use_cartesian_command = true;

//server->client로 조인트 float 값 보낸다
float jointArray[7] = { 0.0 };
float temp[7] = { 0.0 };
bool jointchanged[7] = { false };

int main(int argc, char * argv[]) {
	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	nh.param("joint_position_topic", joint_position_topic, std::string("/iiwa/state/JointPosition"));
	nh.param("cartesian_position_topic", cartesian_position_topic, std::string("/iiwa/state/CartesianPose"));
	nh.param("command_cartesian_position_topic", command_cartesian_position_topic, std::string("/iiwa/command/CartesianPose"));
	nh.param("command_joint_position_topic", command_joint_position_topic, std::string("/iiwa/command/JointPosition"));
	nh.param("use_cartesian_command", use_cartesian_command, true);

	// Dynamic parameter to choose the rate at wich this node should run
	nh.param("ros_rate", ros_rate, 0.5); // 0.1 Hz = 10 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

	// Subscribers and publishers
	ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
	ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
	ros::Publisher pub_cartesian_command = nh.advertise<geometry_msgs::PoseStamped>(command_cartesian_position_topic, 1);
	ros::Publisher pub_joint_command = nh.advertise<iiwa_msgs::JointPosition>(command_joint_position_topic, 1);

	int direction = 1;
	int order = 1;

	int serv_sock;
	char message[BUF_SIZE];//받는 용
	int str_len;
	socklen_t clnt_adr_sz;

	unsigned char floatTobyte[28];
	uint8_t * pointer = (uint8_t *)jointArray;

	struct sockaddr_in serv_adr;
	struct sockaddr_in clnt_adr;

	int i = 0;

	if (argc != 2) {
		printf("Usage : %s <port>\n", argv[0]);
		exit(1);
	}

	serv_sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (serv_sock == -1) {
		error_handling("UDP socket creation error");
	}
	else {
		printf("success to creating UDP socket\n");
	}

	memset(&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family = AF_INET;
	serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_adr.sin_port = htons(atoi(argv[1]));

	if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
		error_handling("bind() error");
	}
	else {
		printf("success to binding\n");
	}

	/*
	while (1) {
	clnt_adr_sz = sizeof(clnt_adr);
	str_len = recvfrom(serv_sock, message, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
	sendto(serv_sock, message, str_len, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz);
	}
	*/

	clnt_adr_sz = sizeof(clnt_adr);
	str_len = recvfrom(serv_sock, message, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
	message[str_len] = 0;
	printf("Received from Client: %s\n", message);

	if (sendto(serv_sock, message, str_len, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1) {
		error_handling("sendto() error");
	}
	else {
		printf("success to sending message\n");
	}

	/*
	jointArray[0] = 90;
	jointArray[1] = 90;
	jointArray[2] = 0;
	jointArray[3] = 0;
	jointArray[4] = 0;
	jointArray[5] = 0;
	jointArray[6] = 0;

	//28byte
	for (i = 0; i<sizeof(jointArray); i++) {//0-27
	floatTobyte[i] = pointer[i];
	if (i % 4 == 0) {
	//ROS_INFO("joint %d", i / 4 + 1);
	printf("joint %d\n", i / 4 + 1);
	}
	//ROS_INFO("------------%x", pointer[i]);
	printf("------------%x\n", pointer[i]);
	}

	if (sendto(serv_sock, floatTobyte, 28, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1) {
	error_handling("sendto() error");
	}
	*/

	while (ros::ok()) {
		if (isRobotConnected) {

			/*jointArray[0] = 90;
			jointArray[1] = 90;
			jointArray[2] = 0;
			jointArray[3] = 0;
			jointArray[4] = 0;
			jointArray[5] = 0;
			jointArray[6] = 0;*/

			jointchanged[7] = { false };

			jointArray[0] = current_joint_position.position.a1;
			jointArray[1] = current_joint_position.position.a2;
			jointArray[2] = current_joint_position.position.a3;
			jointArray[3] = current_joint_position.position.a4;
			jointArray[4] = current_joint_position.position.a5;
			jointArray[5] = current_joint_position.position.a6;
			jointArray[6] = current_joint_position.position.a7;

			deduplication();

			if (jointchanged[0] || jointchanged[1] || jointchanged[2] || jointchanged[3] || jointchanged[4] || jointchanged[5] || jointchanged[6]) {
				//28byte
				for (i = 0; i<sizeof(jointArray); i++) {//0-27
					floatTobyte[i] = pointer[i];
					if (i % 4 == 0) {
						//ROS_INFO("joint %d", i / 4 + 1);
						printf("joint %d\n", i / 4 + 1);
					}
					//ROS_INFO("------------%x", pointer[i]);
					printf("------------%x\n", pointer[i]);
				}

				if (sendto(serv_sock, floatTobyte, 28, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1) {
					error_handling("sendto() error");
				}
			}

			if (use_cartesian_command) {
				// Here we set the new commanded cartesian position, we just move the tool TCP 10 centemeters down and back up, every 20 seconds.
				command_cartesian_position = current_cartesian_position;
				ROS_INFO("command z: %f", command_cartesian_position.pose.position.z);
				switch (order) {
				case 1:
					if (command_cartesian_position.pose.position.z < 0.58632)
						command_cartesian_position.pose.position.z += 0.005;
					else
						order = 2;
					break;
				case 2:
					if (command_cartesian_position.pose.position.y > -0.10000)
						command_cartesian_position.pose.position.y -= 0.005;
					else
						order = 3;
					break;
				case 3:
					if (command_cartesian_position.pose.position.z > 0.38632)
						command_cartesian_position.pose.position.z -= 0.005;
					else
						order = 4;
					break;
				case 4:
					if (command_cartesian_position.pose.position.y < 0.10000)
						command_cartesian_position.pose.position.y += 0.005;
					else
						order = 1;
					break;
				}

				pub_cartesian_command.publish(command_cartesian_position); // Command position is published and executed by the robot (if the robot can achieve that position)

			}
			else {
				//command_joint_position = current_joint_position;
				//command_joint_position.position.a4 -= direction * 0.0872665; // Adding/Subtracting 5 degrees (in radians) to the 4th joint
				//pub_joint_command.publish(command_joint_position); // Command position is published and executed by the robot (if the joint limit is not exceeded)
			}
			//direction *= -1; // In the next iteration the motion will be on the opposite direction
			//loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
		}
		else {
			ROS_ERROR("Robot is not connected!!!!");
			ros::Duration(5.0).sleep(); // 5 seconds
		}
	}//end while

	std::cerr << "Stopping spinner..." << std::endl;
	spinner.stop();
	close(serv_sock);
	std::cerr << "Bye!" << std::endl;

	return 0;
}

void deduplication() {
	if (temp[0] == jointArray[0]) jointchanged[0] = false;
	else
	{
		jointchanged[0] = true;
		temp[0] == jointArray[0];
	}

	if (temp[1] == jointArray[1]) jointchanged[1] = false;
	else
	{
		jointchanged[1] = true;
		temp[1] == jointArray[1];
	}

	if (temp[2] == jointArray[2]) jointchanged[2] = false;
	else
	{
		jointchanged[2] = true;
		temp[2] == jointArray[2];
	}

	if (temp[3] == jointArray[3]) jointchanged[3] = false;
	else
	{
		jointchanged[3] = true;
		temp[3] == jointArray[3];
	}

	if (temp[4] == jointArray[4]) jointchanged[4] = false;
	else
	{
		jointchanged[4] = true;
		temp[4] == jointArray[4];
	}

	if (temp[5] == jointArray[5]) jointchanged[5] = false;
	else
	{
		jointchanged[5] = true;
		temp[5] == jointArray[5];
	}

	if (temp[6] == jointArray[6]) jointchanged[6] = false;
	else
	{
		jointchanged[6] = true;
		temp[6] == jointArray[6];
	}
}

void error_handling(char * message) {
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

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
