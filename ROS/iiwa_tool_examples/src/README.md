# 1. rcvMappingData.cpp

### Purpose of use: Receive a mapping data from HoloLens side.

The mapping data is scanned from HoloLens by spatial mapping. This data will be used for colliding object in Moveit! planning scene.

Usage:
```
  g++ rcvMappingData.cpp -o rcvMdata
  ./rcvMdata
```
In rcvMappingData.cpp, 
1. TCP socket communication is used.
2. Save the mapping data with .txt file.
3. Convert the mapping data in .txt file to .off files.

Addition:

+ In the code, for socket communication, port num: 9192 is used. You can change this number.
+ And all used function is C language so you can change this file to rcvMappingData.c
+ Also you have to change the absolute path of fopen()

### Details:
```C
int ReadInt (unsigned char * message) {...}
```
This function inverts the order of char[] and converts to an integer. Because there is a different byte order between computers which is called host byte order. This can be solved using this function or just using htonl() in sending side and ntohl() in receiving side.
```C
int printOutput (unsigned char * mesh_arr, int byteSize) {...}
```
This function creates a .txt file from received multiple mesh data. In each mesh data, there is 8 byte mesh header which contains the number of vertices(vertexCount) and the number of all indices of triangles(triangleIndexCount). And it has coordinates of all vertices(vertexCount * 3 * 4 byte) and list of 3 indices composing triangles(triangleIndexCount * 4 byte).
```C
int ConvertToOFF () {...}
```
This function creates .off files from a former .txt file. The created files are named with 'environment_mesh_*number*.off'.
Each .off files are beginning with the keyword OFF.
In second line, there are the number of vertices(vertexCount), the number of faces(triangleIndexCount/3), and the number of edges(0).
From the third line, all vertices are listed with x, y, z coordinates. However, because the mesh data follows left-handed coordinate system (because of HoloLens), it needed to be corrected to right-handed coordinate system. So I changed sign of x coordinate.
After the list of coordinates, all faces are listed. For each face, the number of vertices is specified, followed by indices of vertices.

------------
