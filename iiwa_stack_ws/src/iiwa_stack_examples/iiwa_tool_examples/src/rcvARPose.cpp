#include "ros/ros.h"
#include </opt/ros/melodic/include/geometry_msgs/Pose.h>//this should be changed in case
#include </opt/ros/melodic/include/sensor_msgs/JointState.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define BUF_SIZE 100

void error_handling (const char * message);

//float ar_pose_arr[14] = { 0.0 };

//socket programming
int serv_sock;
char message[BUF_SIZE];//received message
int str_len;
socklen_t clnt_adr_sz;

struct sockaddr_in serv_adr;
struct sockaddr_in clnt_adr;
int port = 9191;

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "ARPosePublisher");
	ros::NodeHandle nh;

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher ar_pose_pub = nh.advertise<geometry_msgs::Pose>("ar_pose_from_HoloLens", 100);

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
/*
		//receive hand-shake message("hello")
		clnt_adr_sz = sizeof(clnt_adr);
		str_len = recvfrom(serv_sock, message, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
		message[str_len] = 0;
		printf("Received from Client: %s\n", message);
*/
		//receive start and goal pose
		clnt_adr_sz = sizeof(clnt_adr);
		str_len = recvfrom(serv_sock, message, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
		message[str_len] = 0;

		float spose[7] = { 0.0 };
		float gpose[7] = { 0.0 };

		//float spos_x, spos_y, spos_z, sori_x, sori_y, sori_z, sori_w;
		//float gpos_x, gpos_y, gpos_z, gori_x, gori_y, gori_z, gori_w;
		
		//byte to float
		memcpy(&spose[0], &message[0], 4);
		memcpy(&spose[1], &message[4], 4);
		memcpy(&spose[2], &message[8], 4);
		memcpy(&spose[3], &message[12], 4);
		memcpy(&spose[4], &message[16], 4);
		memcpy(&spose[5], &message[20], 4);
		memcpy(&spose[6], &message[24], 4);

		memcpy(&gpose[0], &message[28], 4);
		memcpy(&gpose[1], &message[32], 4);
		memcpy(&gpose[2], &message[36], 4);
		memcpy(&gpose[3], &message[40], 4);
		memcpy(&gpose[4], &message[44], 4);
		memcpy(&gpose[5], &message[48], 4);
		memcpy(&gpose[6], &message[52], 4);

		spose[2] += 0.84;//z: Add height of base
		spose[0] = spose[0] * -1;//x: handle coordinate change: left handed-> right handed
	
		gpose[2] += 0.84;//z: Add height of base
		gpose[0] = gpose[0] * -1;//x: handle coordinate change: left handed-> right handed

		ROS_INFO("*****************==AR start pose: %f %f %f %f %f %f %f", spose[0], spose[1], spose[2], spose[3], spose[4], spose[5], spose[6]);
		ROS_INFO("*****************==AR goal pose: %f %f %f %f %f %f %f", gpose[0], gpose[1], gpose[2], gpose[3], gpose[4], gpose[5], gpose[6]);

		geometry_msgs::Pose start_pose;
		start_pose.position.x = spose[0];
		start_pose.position.y = spose[1];
		start_pose.position.z = spose[2];
		start_pose.orientation.x= spose[3];			
		start_pose.orientation.y= spose[4];	
		start_pose.orientation.z= spose[5];
		start_pose.orientation.w= spose[6];

		geometry_msgs::Pose goal_pose;
		goal_pose.position.x = gpose[0];
		goal_pose.position.y = gpose[1];
		goal_pose.position.z = gpose[2];
		goal_pose.orientation.x = gpose[3];			
		goal_pose.orientation.y = gpose[4];	
		goal_pose.orientation.z = gpose[5];
		goal_pose.orientation.w = gpose[6];
/*		
		memcpy(&spos_x, &message[0], 4);
		memcpy(&spos_y, &message[4], 4);
		memcpy(&spos_z, &message[8], 4);
		memcpy(&sori_x, &message[12], 4);
		memcpy(&sori_y, &message[16], 4);
		memcpy(&sori_z, &message[20], 4);
		memcpy(&sori_w, &message[24], 4);

		memcpy(&gpos_x, &message[28], 4);
		memcpy(&gpos_y, &message[32], 4);
		memcpy(&gpos_z, &message[36], 4);
		memcpy(&gori_x, &message[40], 4);
		memcpy(&gori_y, &message[44], 4);
		memcpy(&gori_z, &message[48], 4);
		memcpy(&gori_w, &message[52], 4);

		spos_z += 0.84;//Add height of base
		spos_x = spos_x*-1;//handle coordinate change: left handed-> right handed
	
		gpos_z += 0.84;//Add height of base
		gpos_x = gpos_x*-1;//handle coordinate change: left handed-> right handed

		ROS_INFO("*****************==AR start pose: %f %f %f %f %f %f %f", spos_x, spos_y, spos_z, sori_x, sori_y, sori_z, sori_w);
		ROS_INFO("*****************==AR goal pose: %f %f %f %f %f %f %f", gpos_x, gpos_y, gpos_z, gori_x, gori_y, gori_z, gori_w);
	
		geometry_msgs::Pose start_pose;
		start_pose.position.x = spos_x;
		start_pose.position.y = spos_y;
		start_pose.position.z = spos_z;
		start_pose.orientation.x= sori_x;			
		start_pose.orientation.y= sori_y;	
		start_pose.orientation.z= sori_z;
		start_pose.orientation.w= sori_w;

		geometry_msgs::Pose goal_pose;
		goal_pose.position.x = gpos_x;
		goal_pose.position.y = gpos_y;
		goal_pose.position.z = gpos_z;
		goal_pose.orientation.x= gori_x;			
		goal_pose.orientation.y= gori_y;	
		goal_pose.orientation.z= gori_z;
		goal_pose.orientation.w= gori_w;
*/	
		ar_pose_pub.publish(start_pose);
		ar_pose_pub.publish(goal_pose);

	
		ROS_INFO("*****************==AR poses are published!************************");

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
