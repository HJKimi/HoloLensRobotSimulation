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

Details:
### int ReadInt (char * message)


------------
