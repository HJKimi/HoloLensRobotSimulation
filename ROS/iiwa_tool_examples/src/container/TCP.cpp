#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define BUF_SIZE 1024
//#define buff_leng 64512

void error_handling (const char * message);
int ReadInt (char * message);
int printOutput (unsigned char * mesh_arr, int size);


unsigned char * mesh_arr = 0;
int datasize = 0;
FILE *ofp;


int main (int argc, char * argv[]) {
	int serv_sock, clnt_sock;
	char message[BUF_SIZE];
	int str_len, recv_len;

	struct sockaddr_in serv_adr;
	struct sockaddr_in clnt_adr;
	socklen_t clnt_adr_sz;
	

	if (argc != 2) {
 		printf("Usage : %s <port>\n", argv[0]);
 		exit(1);
  	}

	serv_sock = socket (PF_INET, SOCK_STREAM, 0); //create socket
	if (serv_sock == -1)
	{
		error_handling ("TCP socket creation error");
	}
	else
	{
		printf("Success to creating TCP socket\n");
	}

	memset (&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family = AF_INET;
	serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_adr.sin_port = htons(atoi(argv[1]));

	if (bind (serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1)
	{
		error_handling ("bind() error");
	}
	else
	{
		printf("Success to binding\n");
	}

	if (listen(serv_sock, 5) == -1)
	{
		error_handling("listen() error");
	}
	else
	{
		printf("Success to listening\n");
	}

	clnt_adr_sz = sizeof(clnt_adr);
	clnt_sock = accept(serv_sock, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);

	if (clnt_sock == -1)
	{
		error_handling("accept() error");
	}

  	printf("waiting for message from client\n");
	
	
	
	//Receive 4 byte: (int) datasize
	str_len = read(clnt_sock, message, 4);
	message[str_len] = 0;

	datasize = ReadInt (message);
	printf("Mesh data size = %d\n", datasize);

	//Create mesh data array
	mesh_arr = (unsigned char *)malloc(sizeof(unsigned char)*(datasize + 1));

	//Receive mesh data
	recv_len = 0;	
	while (recv_len < datasize) {
		recv_len += read(clnt_sock, &mesh_arr[recv_len], datasize);
	}
	mesh_arr[recv_len] = 0;
	printf("Mesh data received!\n");

	//Print mesh data
	if (printOutput(mesh_arr, datasize) != 1) {
		printf("Failed to print output!\n");
	} else {
		printf("Print out mapping data\n");
        }

	free(mesh_arr);
	close(clnt_sock);
	close(serv_sock);

	return 0;
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

int printOutput (unsigned char * mesh_arr, int byteSize) {
	int i;
	int vertexCount,triangleIndexCount;
	float vx, vy, vz;
	int index1, index2, index3;
	int count = 0;

	ofp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/mapping_data.txt", "w");
  	if(!ofp) {
    		printf("Can't open output file\n");
		return -1;
	}

	while (count<byteSize) {
		//memcpy(&vertexCount, &mesh_arr[0], sizeof(vertexCount));
		memcpy(&vertexCount, &mesh_arr[count], sizeof(vertexCount));
		fprintf(ofp, "%d\n", vertexCount);

		memcpy(&triangleIndexCount, &mesh_arr[count + 4], sizeof(triangleIndexCount));
		fprintf(ofp, "%d\n", triangleIndexCount);

		const int indexCountOffset = 4*2;

		for (int i=0; i<vertexCount; i++) {
			memcpy(&vx, &mesh_arr[count + 12*i + indexCountOffset  ], sizeof(vx));
			memcpy(&vy, &mesh_arr[count + 12*i + indexCountOffset+4], sizeof(vy));
			memcpy(&vz, &mesh_arr[count + 12*i + indexCountOffset+8], sizeof(vz));
			fprintf(ofp, "%f ", vx);
			fprintf(ofp, "%f ", vy);
			fprintf(ofp, "%f\n", vz);
		}

		for (int i=0; i<triangleIndexCount/3; i++) {
			//memcpy(&index1, &mesh_arr[4*(2 + 3*vertexCount + 3*i)], sizeof(index1));
			memcpy(&index1, &mesh_arr[count + 12*vertexCount + 12*i + indexCountOffset  ], sizeof(index1));
			memcpy(&index2, &mesh_arr[count + 12*vertexCount + 12*i + indexCountOffset+4], sizeof(index2));
			memcpy(&index3, &mesh_arr[count + 12*vertexCount + 12*i + indexCountOffset+8], sizeof(index3));
			fprintf(ofp, "%d ", index1);
			fprintf(ofp, "%d ", index2);
			fprintf(ofp, "%d\n", index3);
		}

		count += 4*(2 + 3*vertexCount + triangleIndexCount);
	}

	fclose(ofp);
	return 1;
}
