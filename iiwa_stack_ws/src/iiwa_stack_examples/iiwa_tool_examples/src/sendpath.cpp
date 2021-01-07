#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

//#include "ros/ros.h"
//#include "iiwa_msgs/JointPosition.h"
//#include "geometry_msgs/PoseStamped.h"

#define BUF_SIZE 100

void error_handling (const char * message);
int readInput ();

//iiwa_msgs::JointPosition current_joint_position, command_joint_position;
//geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
//std::string joint_position_topic, cartesian_position_topic, command_cartesian_position_topic, command_joint_position_topic;
//double ros_rate = 0.1;
//bool isRobotConnected = false, use_cartesian_command = true;

FILE *ifp;
double * pos_arr = 0;
int size;

//server->client로 조인트 float 값 보낸다
float jointArray[7] = { 0.0 };
float temp[7] = { 0.0 };
bool jointchanged[7] = { false };

int main(int argc, char * argv[]) {
	int loop = 0;

	int serv_sock;
	char message[BUF_SIZE];//받는 용
	int str_len;
	socklen_t clnt_adr_sz;

	unsigned char floatTobyte[28];
	uint8_t * pointer = (uint8_t *)jointArray;

	struct sockaddr_in serv_adr;
	struct sockaddr_in clnt_adr;
	int port = 9193;

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

	while (1) {
		//rcv 'hello'
		clnt_adr_sz = sizeof(clnt_adr);
		str_len = recvfrom(serv_sock, message, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);
		message[str_len] = 0;
		printf("Received from Client: %s\n", message);

		//send 'hello'
		if (sendto(serv_sock, message, str_len, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1) {
			error_handling("sendto() error");
		}
		else {
			printf("success to sending message\n");
		}

		if (readInput () != 1) {
			printf("Failed to read input!\n");
		} else {
			printf("Succeded to read input\n");
		}

		loop = 0;

		while (loop != (size/7)) {
			//convert double to float
			jointArray[0] = (float)pos_arr[loop*7];
			jointArray[1] = (float)pos_arr[loop*7+1];
			jointArray[2] = (float)pos_arr[loop*7+2];
			jointArray[3] = (float)pos_arr[loop*7+3];
			jointArray[4] = (float)pos_arr[loop*7+4];
			jointArray[5] = (float)pos_arr[loop*7+5];
			jointArray[6] = (float)pos_arr[loop*7+6];

			printf("=================================State %d\n", loop+1);

			for (int i = 0; i<sizeof(jointArray); i++) {//0-27
				floatTobyte[i] = pointer[i];
				/*
				if (i % 4 == 0) {
					printf("joint %d\n", i / 4 + 1);
				}
				printf("------------%x\n", pointer[i]);
				*/
			}

			if (sendto(serv_sock, floatTobyte, 28, 0, (struct sockaddr *)&clnt_adr, clnt_adr_sz) == -1) {
				error_handling("sendto() error");
			}

			usleep(1400000);

			loop++;
		}

		free(pos_arr);
	}

	close(serv_sock);
	
	return 0;
}

void error_handling (const char * message) {
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

int readInput () {
	int n=0;

	ifp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/planning_output.txt", "r");
	if(!ifp) {
		printf("Can't open a file\n");
		return -1;
	}

	fscanf(ifp, "%d", &size);
	pos_arr = (double *)malloc(sizeof(double)*size);

	while(!feof(ifp)) {
		fscanf(ifp, "%lf", &pos_arr[n]);
		n++;
	}
/*
	printf("%d\n", size);
	for (int i=0; i<size; i++) {
    printf("%f ", pos_arr[i]);
  }
*/

	fclose(ifp);
	return 1;
}