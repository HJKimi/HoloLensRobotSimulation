#include "ros/ros.h"
#include </opt/ros/indigo/include/geometry_msgs/Pose.h>
#include </opt/ros/indigo/include/sensor_msgs/JointState.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define BUF_SIZE 100

void error_handling (const char * message);

//socket programming
int serv_sock;
char message[BUF_SIZE];//received message
int str_len;
socklen_t clnt_adr_sz;

struct sockaddr_in serv_adr;
struct sockaddr_in clnt_adr;
int port = 9194;

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "MoveCommandPublisher");
	ros::NodeHandle nh;

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher comm_pub = nh.advertise<geometry_msgs::Pose>("move_command_from_HoloLens", 100);

	//ros::Rate loop_rate(1);//1sec
	ros::Rate loop_rate(100);//0.01sec

	serv_sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (serv_sock == -1) {
		error_handling("UDP socket creation error");
	}
	else {
		printf("success to creating UDP socket\n");
	}

	int enable = 1;
	if (setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    		error_handling("setsockopt(SO_REUSEADDR) failed");
	}

	memset(&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family = AF_INET;
	serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_adr.sin_port = htons(port);

	if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
		error_handling("bind() error");
	}
	else {
		printf("success to binding\n");
	}

	while (ros::ok()) {
		//receive hand-shake message("hello")
		clnt_adr_sz = sizeof(clnt_adr);
		str_len = recvfrom(serv_sock, message, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
		message[str_len] = 0;
		printf("Received from Client: %s\n", message);

		geometry_msgs::Pose msg;
		comm_pub.publish(msg);

		ROS_INFO("*****************move command is published!************************");

		loop_rate.sleep();
	}

	close(serv_sock);

	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;
}

void error_handling (const char * message) {
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}
