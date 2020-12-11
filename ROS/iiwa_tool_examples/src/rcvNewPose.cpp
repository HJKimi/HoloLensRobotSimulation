#include "ros/ros.h"
#include </opt/ros/indigo/include/geometry_msgs/Pose.h>
#include </opt/ros/indigo/include/sensor_msgs/JointState.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define BUF_SIZE 1024

void jointstateCallback(const sensor_msgs::JointState& new_state);
void error_handling (const char * message);
int ReadInt (char * message);

sensor_msgs::JointState eef_joint_state;
bool rcvState = false;

//receive from HoloLens
float pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;

//send to HoloLens
unsigned char sendM[28];
uint8_t * pointer;


//socket programming
int serv_sock;
char message[BUF_SIZE];//received message
int str_len;
socklen_t clnt_adr_sz;

struct sockaddr_in serv_adr;
struct sockaddr_in clnt_adr;
int port = 9190;


int main (int argc, char * argv[]) {	
	ros::init(argc, argv, "PoseMsgPublisher");
	ros::NodeHandle nh;

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("pose_msg_from_HoloLens", 100);
	ros::Subscriber pose_sub = nh.subscribe("/iiwa/CommandRobotMoveit/jointstate_from_pose", 10, jointstateCallback);

	//ros::Rate loop_rate(1);//1sec
	ros::Rate loop_rate(100);//0.01sec

	int i = 0;
/*
	if (argc != 2) {
		printf("Usage : %s <port>\n", argv[0]);
		exit(1);
	}
*/
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
	//serv_adr.sin_port = htons(atoi(argv[1]));
	serv_adr.sin_port = htons(port);

	if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
		error_handling("bind() error");
	}
	else {
		printf("success to binding\n");
	}

	//receive hand-shake message("hello")
	clnt_adr_sz = sizeof(clnt_adr);
	str_len = recvfrom(serv_sock, message, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
	message[str_len] = 0;
	printf("Received from Client: %s\n", message);


	while (ros::ok()) {
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
		
		//printf("Received from Client: %s\n", message);


		float pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;

		//byte to float
		memcpy(&pos_x, &message[0], 4);
		memcpy(&pos_y, &message[4], 4);
		memcpy(&pos_z, &message[8], 4);
		memcpy(&ori_x, &message[12], 4);
		memcpy(&ori_y, &message[16], 4);
		memcpy(&ori_z, &message[20], 4);
		memcpy(&ori_w, &message[24], 4);

		pos_z += 0.84;//Add height of base
		pos_x = pos_x*-1;//handle coordinate change: left handed-> right handed


		geometry_msgs::Pose new_pose;
		new_pose.position.x = pos_x;
		new_pose.position.y = pos_y;
		new_pose.position.z = pos_z;
		new_pose.orientation.x= ori_x;			
		new_pose.orientation.y= ori_y;	
		new_pose.orientation.z= ori_z;
		new_pose.orientation.w= ori_w;

		ROS_INFO("*************************Received new pose: %f %f %f %f %f %f %f\n", new_pose.position.x, new_pose.position.y, new_pose.position.z, new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w);

		pose_pub.publish(new_pose);

		if (rcvState) {
			//float to byte: 28byte
			for (int i=0; i<7; i++) {
				float temp = (float)eef_joint_state.position[i];//float64 to float
				pointer = (uint8_t *)&temp;
				for (int j=0; j<4; j++) {
					sendM[4*i+j] = pointer[j];
				}
			}


			if (sendto(serv_sock, sendM, 28, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1) {
				error_handling("sendto() error");
			}
			else {
				printf("success to sending message\n");
			}
		}


/*
		geometry_msgs::Pose new_pose;
		new_pose.position.x = 0.6535;
		new_pose.position.y = 0.0;
		new_pose.position.z = 1.58;
		new_pose.orientation.x= 0.0;			
		new_pose.orientation.y= 0.0;	
		new_pose.orientation.z= 0.0;
		new_pose.orientation.w= 1;

		ROS_INFO("new pose: %f %f %f %f %f %f %f\n", new_pose.position.x, new_pose.position.y, new_pose.position.z, new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w);
		pose_pub.publish(new_pose);
*/		

		
		


		//sendM = (float) eef_joint_state.position[0];
		//(float)eef_joint_state.position[1]
		//(float)eef_joint_state.position[2]
		//(float)eef_joint_state.position[3]
		//(float)eef_joint_state.position[4]
		//(float)eef_joint_state.position[5]
		//(float)eef_joint_state.position[6]
		

		



		
		




		loop_rate.sleep();
	}

	close(serv_sock);

	//close(clnt_sock);
	//close(serv_sock);

	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;
}

void jointstateCallback(const sensor_msgs::JointState& new_state) {
	if (!rcvState)
		rcvState = !rcvState;
	eef_joint_state = new_state;
	
	//ROS_INFO("new joint state: %f %f %f %f %f %f %f\n", eef_joint_state.position[0], eef_joint_state.position[1], eef_joint_state.position[2], eef_joint_state.position[3], eef_joint_state.position[4], eef_joint_state.position[5], eef_joint_state.position[6]);
	
/*
	//float to byte: 28byte
	for (int i=0; i<7; i++) {
		float temp = (float)eef_joint_state.position[i];//float64 to float
		pointer = (uint8_t *)&temp;
		for (int j=0; j<4; j++) {
			sendM[4*i+j] = pointer[j];
		}
	}

	if (sendto(serv_sock, sendM, 28, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1) {
		error_handling("sendto() error");
	}
	else {
		printf("success to sending message\n");
	}
*/
}

void error_handling (const char * message) {
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

int ReadInt (char * message) {
	unsigned char bytes[4];
	unsigned char t;
	int * datasize;

	memcpy(bytes, message, sizeof(bytes));

	t = bytes[0];
	bytes[0] = bytes[3];
	bytes[3] = t;
	
	t = bytes[1];
	bytes[1] = bytes[2];
	bytes[2] = t;

	datasize = (int *)bytes;

	return *datasize;
}
