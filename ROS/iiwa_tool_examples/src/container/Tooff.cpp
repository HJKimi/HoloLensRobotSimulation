#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <string>
//#include <unistd.h>
//#include <cstdio>

int ConvertToOFF ();

FILE *ifp;
FILE *ofp;

int main() {
	if (ConvertToOFF () != 1) {
		printf("Failed to print output!\n");
	} else {
		printf("Print out mapping data\n");
        }

	return 0;
}



int ConvertToOFF () {
	int vertexCount,triangleIndexCount;
	float vx, vy, vz;
	int index1, index2, index3;
	float * vertex = 0;
	int * index = 0;
	char dir[1024];
	int loopNum = 0;


	ifp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/mapping_data.txt", "r");
	if(!ifp) {
		printf("Can't open a file\n");
		return -1;
	}

	while (!feof(ifp)) {
		char a[64];

		sprintf(a, "%d", loopNum);
		strcpy (dir, "/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/meshes/environment_mesh_");
		strcat (dir, a);
		strcat (dir, ".off");

		ofp = fopen(dir, "w");
  		if(!ofp) {
    			printf("Can't open output file: file %d\n", loopNum);
			return -1;
		}

		fprintf(ofp, "OFF\n");
	
		fscanf(ifp, "%d", &vertexCount);
		fprintf(ofp, "%d ", vertexCount);

		fscanf(ifp, "%d", &triangleIndexCount);
		fprintf(ofp, "%d ", triangleIndexCount/3);

		fprintf(ofp, "0\n");
	
		vertex = (float *)malloc(sizeof(float) * vertexCount * 3);
		for (int i=0; i<vertexCount*3; i++) {
			fscanf(ifp, "%f", &vertex[i]);
		}

		for (int i=0; i<vertexCount; i++) {
			memcpy(&vx, &vertex[3*i], sizeof(vx));
			memcpy(&vy, &vertex[3*i+1], sizeof(vy));
			memcpy(&vz, &vertex[3*i+2], sizeof(vz));
			fprintf(ofp, "%f ", -vx);
			fprintf(ofp, "%f ", vy);
			fprintf(ofp, "%f\n", vz);
		}

		index = (int *)malloc(sizeof(int) * triangleIndexCount);
		for (int i=0; i<triangleIndexCount; i++) {
			fscanf(ifp, "%d", &index[i]);
		}

		for (int i=0; i<triangleIndexCount/3; i++) {
			memcpy(&index1, &index[3*i], sizeof(index1));
			memcpy(&index2, &index[3*i + 1], sizeof(index2));
			memcpy(&index3, &index[3*i + 2], sizeof(index3));
			fprintf(ofp, "3 ");
			fprintf(ofp, "%d ", index1);
			fprintf(ofp, "%d ", index2);
			fprintf(ofp, "%d\n", index3);
		}

		free(index);
		free(vertex);
		fclose(ofp);
		loopNum++;
	}
	
	fclose(ifp);
/*
	ifp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/mapping_data.txt", "r");
	if(!ifp) {
		printf("Can't open a file\n");
		return -1;
	}

	ofp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/mapping_data_toOFF.off", "w");
  	if(!ofp) {
    		printf("Can't open output file\n");
		return -1;
	}

	fprintf(ofp, "OFF\n");
	
	fscanf(ifp, "%d", &vertexCount);
	fprintf(ofp, "%d ", vertexCount);

	fscanf(ifp, "%d", &triangleIndexCount);
	fprintf(ofp, "%d ", triangleIndexCount/3);

	fprintf(ofp, "0\n");
	
	vertex = (float *)malloc(sizeof(float) * vertexCount * 3);
	for (int i=0; i<vertexCount*3; i++) {
		fscanf(ifp, "%f", &vertex[i]);
	}

	for (int i=0; i<vertexCount; i++) {
		memcpy(&vx, &vertex[3*i], sizeof(vx));
		memcpy(&vy, &vertex[3*i+1], sizeof(vy));
		memcpy(&vz, &vertex[3*i+2], sizeof(vz));
		fprintf(ofp, "%f ", vx);
		fprintf(ofp, "%f ", vy);
		fprintf(ofp, "%f\n", vz);
	}

	index = (int *)malloc(sizeof(int) * triangleIndexCount);
	for (int i=0; i<triangleIndexCount; i++) {
		fscanf(ifp, "%d", &index[i]);
	}

	for (int i=0; i<triangleIndexCount/3; i++) {
		memcpy(&index1, &index[3*i], sizeof(index1));
		memcpy(&index2, &index[3*i + 1], sizeof(index2));
		memcpy(&index3, &index[3*i + 2], sizeof(index3));
		fprintf(ofp, "3 ");
		fprintf(ofp, "%d ", index1);
		fprintf(ofp, "%d ", index2);
		fprintf(ofp, "%d\n", index3);
	}

	free(index);
	free(vertex);
	fclose(ofp);
	fclose(ifp);

*/
	return 1;
}
