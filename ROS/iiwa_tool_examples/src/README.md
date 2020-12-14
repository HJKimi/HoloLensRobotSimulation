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
int ReadInt (char * message) {...}
```
This function inverts the order of char[] and converts to an integer. Because there is a different byte order between computers which is called host byte order. This can be solved using this fuction or just using htonl() in sending side and ntohl() in receiving side.
```C
int printOutput (unsigned char * mesh_arr, int byteSize) {...}
```
This function creats a .txt file from received multiple mesh data. In each mesh data, there is 8 byte mesh header which contains the number of vertices(vertexCount) and the number of all indices of trianlges(triangleIndexCount). And it has coordinates of all vertices(vertexCount * 3 * 4 byte) and list of 3 indicecs composing triangles(triangleIndexCount * 4 byte).

------------
