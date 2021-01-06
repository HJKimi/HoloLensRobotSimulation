//Joint angle target input

#include <stdio.h>
#include <vector>

#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"

//directory of below: '/opt/ros/melodic/include/~'
//#include <moveit/move_group_interface/move_group.h>//original

//for MoveGroupInterface
#include <moveit/move_group_interface/move_group_interface.h>//changed

//for PlanningSceneInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <moveit/planning_interface/planning_interface.h>//not used

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/RobotTrajectory.h>


#include "numofmesh.h"

using namespace std;


/*---for choosing input mode---*/
/*
mode 1: Mouse input in RviZ
mode 2: TbD input with Robot
mode 3: AR input in HoloLens */
int mode = 1;

/*---variables of mode 1: input by Rviz---*/
moveit_msgs::DisplayTrajectory current_display_trajectory;//hj
bool rcvtra = false;//for checking whether dispaly trajectory is received or not

/*---variables of mode 2: TbD---*/
bool init = false;
int setInput = 0;
bool printStart = false;//for check the num of printing
bool once = false;//for isRobotConnected==true
bool once2 = false;//for !isRobotConnected

/*---variables of mode 3: Give inputs through AR---*/
std::vector<double> group_pos_arr;//JointGroupPositions of the RobotState
geometry_msgs::Pose eef_pose;//specified pose for tool-tip
sensor_msgs::JointState js_from_pose;
bool rcvarstartpose = false;
bool rcvargoalpose = false;
geometry_msgs::Pose ar_start_pose;
geometry_msgs::Pose ar_goal_pose;
bool once4 = false;

/*---variables for print out a trajectory---*/
FILE *ofp;//for print a path in form of .txt
FILE *ifp;//for read meshNum

/*---variable for moving robot---*/
bool rcvmovecommand = false;

/*---variables for environment mesh---*/
bool co_load = false;//checking CollisionObject is loaded
int meshNum;
//extern int meshNum2;

double ros_rate = 0.1;
bool isRobotConnected = false;

//for planning
std::string joint_position_topic, cartesian_position_topic, movegroup_name, ee_link;
iiwa_msgs::JointPosition current_joint_position;
geometry_msgs::PoseStamped current_cartesian_position;
moveit_msgs::RobotTrajectory trajectory;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps);
void trajectoryCallback(const moveit_msgs::DisplayTrajectory& dp);
void poseCallback(const geometry_msgs::Pose& new_pose);
void ARposeCallback (const geometry_msgs::Pose& ar_pose);
void moveCommandCallback (const geometry_msgs::Pose& msg);
int printOutput (double * pos_arr, int size);
int readMeshNum ();//note the path

int main (int argc, char **argv) {
        //printf("==========!! extern test : %d\n", meshNum2);


  	ROS_INFO("*******************MOVEIT CODE RUN*******************");
	// Initialize ROS
	ros::init(argc, argv, "CommandRobotMoveit");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	nh.param<std::string>("joint_position_topic", joint_position_topic, "/iiwa/state/JointPosition");
	nh.param<std::string>("cartesian_position_topic", cartesian_position_topic, "/iiwa/state/CartesianPose");
	nh.param<std::string>("move_group", movegroup_name, "manipulator");
	//nh.param<std::string>("ee_link", ee_link, "tool_link");
        nh.param<std::string>("ee_link", ee_link, "tool_link_ee");//coordinate frame of tip of tool

	// Dynamic parameter to choose the rate at wich this node should run
	nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

	// Subscribers and publishers
	ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
	ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
	//ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	//ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 100);
	
	/*---for mode 1---*/
	ros::Subscriber sub_planned_trajectory = nh.subscribe("/iiwa/move_group/display_planned_path", 3, trajectoryCallback);

	/*---for mode 3---*/
	ros::Subscriber sub_pose = nh.subscribe("/iiwa/pose_msg_from_HoloLens", 10, poseCallback);
	ros::Publisher pub_state = nh.advertise<sensor_msgs::JointState>("jointstate_from_pose", 100);
	///////ros::Publisher pub_state = nh.advertise<sensor_msgs::JointState>("/iiwa/CommandRobotMoveit/jointstate_from_pose", 100);
	ros::Subscriber ar_pose_sub = nh.subscribe("/iiwa/ar_pose_from_HoloLens", 10, ARposeCallback);
	ros::Subscriber move_command_sub = nh.subscribe("/iiwa/move_command_from_HoloLens", 10, moveCommandCallback);
	ros::Rate loop_rate(100);//0.01sec


	// Create MoveGroup
        //move_group_interface::MoveGroup group(movegroup_name);//original

        //MoveGroupInterface class (move_group_interface.h) can be easily setup
        //using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface group(movegroup_name);//changed


        //moveit::planning_interface::MoveGroup::Plan myplan;//original
        //MoveGroupInterface class (move_group_interface.h)
        moveit::planning_interface::MoveGroupInterface::Plan myplan;//changed

        // Use the PlanningSceneInterface class (planning_scene_interface.h)
        // to add and remove collision objects in our "virtual world" scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Configure planner
	//group.setPlanningTime(0.5);
        group.setPlanningTime(5.0);
        group.setPlannerId("RRTConnectkConfigDefault");
	//group.setPlannerId("RRTstarkConfigDefault");
	//group.setPlannerId("RRTkConfigDefault");
	/*
	  planner_configs:
	    - SBLkConfigDefault
	    - ESTkConfigDefault
	    - LBKPIECEkConfigDefault
	    - BKPIECEkConfigDefault
	    - KPIECEkConfigDefault
	    - RRTkConfigDefault
	    - RRTConnectkConfigDefault
	    - RRTstarkConfigDefault
	    - TRRTkConfigDefault
	    - PRMkConfigDefault
	    - PRMstarkConfigDefault
	*/
        group.setEndEffectorLink(ee_link);

	
	std::vector<double> group_variable_values1, group_variable_values2;
	bool success = false;
	
	if (readMeshNum () != 1) {
                ROS_INFO("Failed to read num of meshes!\n");
                //printf("Failed to read num of meshes!\n");
	} else {
            ROS_INFO("Read num of meshes\n");
                //printf("Read num of meshes\n");
        }




	//mode=3;
	while (ros::ok()) {
		//ROS_INFO("*****************num of meshes: %d", meshNum);	
		if (!co_load) {
                        ROS_INFO("*****************Reference frame: %s", group.getPlanningFrame().c_str());
                        ROS_INFO("*****************Reference frame: %s", group.getEndEffectorLink().c_str());

			moveit_msgs::CollisionObject collision_object;
                        collision_object.header.frame_id = group.getPlanningFrame();
			collision_object.id = "environment";

                        //for (int i=0; i<meshNum; i++) {
                        for (int i=0; i<10; i++) {
				char dir[1024];
				char a[64];

				sprintf(a, "%d", i);
                                strcpy (dir, "file:///home/hj/iiwa_stack_ws/src/iiwa_stack_examples/iiwa_tool_examples/src/meshes/environment_mesh_");
				strcat (dir, a);
				strcat (dir, ".off");

				shapes::Mesh* m = shapes::createMeshFromResource(dir);
				shape_msgs::Mesh mesh;
		    		shapes::ShapeMsg mesh_msg;
		    		shapes::constructMsgFromShape(m, mesh_msg);
		    		mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

				//geometry_msgs::Pose mesh_pose;
				//mesh_pose.position.x = -0.3;
                                // //mesh_pose.position.y = -0.3;
				//mesh_pose.position.z = 0.8;

				geometry_msgs::Pose mesh_pose;
				mesh_pose.position.x = 0.165;
				//mesh_pose.position.y = -0.3;
				mesh_pose.position.z = 0.939;

				collision_object.meshes.push_back(mesh);
				collision_object.mesh_poses.push_back(mesh_pose);
				collision_object.operation = collision_object.ADD;
			}

/*
//bunny
Eigen::Vector3d scale(0.01, 0.01, 0.01);
shapes::Mesh* m = shapes::createMeshFromResource("file:///home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/other_meshes/bunny.obj", scale);
shape_msgs::Mesh mesh;
shapes::ShapeMsg mesh_msg;
shapes::constructMsgFromShape(m, mesh_msg);
mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
geometry_msgs::Pose mesh_pose;
mesh_pose.position.x = 0.6;
mesh_pose.position.z = 0.0;
//mesh_pose.position.z = 1.0;
mesh_pose.orientation.w= 0.7071068;
mesh_pose.orientation.x= 0.7071068;
mesh_pose.orientation.y= 0.0;
mesh_pose.orientation.z= 0.0;
collision_object.meshes.push_back(mesh);
collision_object.mesh_poses.push_back(mesh_pose);
collision_object.operation = collision_object.ADD;
//
*/
			std::vector<moveit_msgs::CollisionObject> collision_objects;
			collision_objects.push_back(collision_object);

			ROS_INFO("*****************Add an object into the world");
			//planning_scene_interface.addCollisionObjects(collision_objects);
			planning_scene_interface.applyCollisionObjects(collision_objects);

			sleep(5.0);
			co_load = true;
		}

		if (mode == 3) {
			if (!rcvmovecommand) {
                                bool set = group.setJointValueTarget(eef_pose, ee_link);
				//ROS_INFO("*****************setJointValueTarget(): %s", set ? "success" : "FAILED");

				if (set) {
					robot_state::RobotState current_state(group.getJointValueTarget());
					//current_state.printStateInfo(std::cout);
				
					robotStateToJointStateMsg(current_state, js_from_pose);	
					pub_state.publish(js_from_pose);

					//current_state.copyJointGroupPositions(group.getRobotModel()->getJointModelGroup(movegroup_name), group_pos_arr);
				
					/*
					printf("position: ");			
					for (int i=0; i< group_pos_arr.size(); i++) {	
						printf("%f\t", group_pos_arr[i]);
					}
					printf("\n");
					*/
				}
			}
			
			if (rcvarstartpose && rcvargoalpose) {
			//if (!once4) {
				ROS_INFO("*****************Start to plan with AR start/goal pose!************************");

                                group.setJointValueTarget(ar_start_pose, ee_link);
                                robot_state::RobotState move_start_state(group.getJointValueTarget());
                                group.setStartState(move_start_state);

                                group.setJointValueTarget(ar_goal_pose, ee_link);
                                //success = group.plan(myplan);//original
                                success = (group.plan(myplan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO("*****************Visualizing plan 3 (pose goal) %s",success?"":"FAILED");
				sleep(5.0);


				if (success) {
					ROS_INFO("*****************EXECUTE");
					//group.execute(myplan);

					trajectory = myplan.trajectory_;

					std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points;
					trajectory_points = trajectory.joint_trajectory.points;

					std::vector<int>::size_type vectorSize = trajectory_points.size();

					std::vector<double> pos_arr;
					int size;

					//put the all joint values to pos_arr vector
					for (int i=0; i<vectorSize; i++) {
						pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[0]);
						pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[1]);
						pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[2]);
						pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[3]);
						pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[4]);
						pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[5]);
						pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[6]);
					}
					size = pos_arr.size();

					if (printOutput(&pos_arr[0], size) != 1) {
						printf("Failed to print output!\n");
					} else {
						printf("*****************Print out planned path\n");
					}
				}

				rcvarstartpose = false;
				rcvargoalpose = false;
				//once4 = true;
			}

			if (rcvmovecommand) {
				ROS_INFO("*****************Inside if (rcvmovecommand) {}");
                                group.setJointValueTarget(ar_start_pose, ee_link);
                                robot_state::RobotState move_start_state(group.getJointValueTarget());
                                group.setStartState(move_start_state);

                                group.setJointValueTarget(ar_goal_pose, ee_link);
                                //success = group.plan(myplan);//original
                                success = (group.plan(myplan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO("*****************Visualizing plan 3 (pose goal) %s",success?"":"FAILED");
				sleep(5.0);

				if (success) {
					// sleep(2.0);
                                        group.move();
				}

				rcvmovecommand = false;
			}

			loop_rate.sleep();
		}//mode == 3
		
		//mouse input
		if (mode == 1) {
			if (rcvtra) {
				trajectory = current_display_trajectory.trajectory[0];
				std::vector<trajectory_msgs::JointTrajectoryPoint> rviz_trajectory_points;
				rviz_trajectory_points = trajectory.joint_trajectory.points;
				int pointnum = 0;
 				//pointnum = trajectory.joint_trajectory.points.size();
				pointnum = rviz_trajectory_points.size();
				std::vector<double> rviz_pos_arr;
								
				for (int i=0; i<pointnum; i++) {
					for (int j=0; j<7; j++) {
						//printf("%f\t", trajectory.joint_trajectory.points[i].positions[j]);
						rviz_pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[0]);
					}
					//printf("\n");
				}
				
				if (printOutput(&rviz_pos_arr[0], rviz_pos_arr.size()) != 1) {
					printf("Failed to print output!\n");
				} else {
					printf("*****************Print out planned path\n");
				}

				rcvtra = false;
			}

			if (rcvmovecommand) {
				ROS_INFO("*****************Inside if (rcvmovecommand) {}");
				/*
				group.setJointValueTarget(ar_start_pose, ee_link);
				robot_state::RobotState move_start_state(group.getJointValueTarget());
				group.setStartState(move_start_state);

				group.setJointValueTarget(ar_goal_pose, ee_link);
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 3 (pose goal) %s",success?"":"FAILED");
				sleep(5.0);

				if (success) {
					// sleep(2.0);
					group.move();
				}
				*/
				rcvmovecommand = false;
			}

			loop_rate.sleep();
		}//mode == 1

		//TbD
		if (mode == 2) {
			//setting start and goal configuration
			if(isRobotConnected && !init) {
				if(setInput==1 && !printStart) {
					ROS_INFO("Current Joint Position is : [ %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]",
					       current_joint_position.position.a1,
					       current_joint_position.position.a2,
					       current_joint_position.position.a3,
					       current_joint_position.position.a4,
					       current_joint_position.position.a5,
					       current_joint_position.position.a6,
					       current_joint_position.position.a7);

					group_variable_values1.push_back(current_joint_position.position.a1);
					group_variable_values1.push_back(current_joint_position.position.a2);
					group_variable_values1.push_back(current_joint_position.position.a3);
					group_variable_values1.push_back(current_joint_position.position.a4);
					group_variable_values1.push_back(current_joint_position.position.a5);
					group_variable_values1.push_back(current_joint_position.position.a6);
					group_variable_values1.push_back(current_joint_position.position.a7);

					ROS_INFO("**********GET START JOINT PARAM");
					printStart = true;
		      		}
				else if(setInput==2) {
					ROS_INFO("Current Joint Position is : [ %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]",
					       current_joint_position.position.a1,
					       current_joint_position.position.a2,
					       current_joint_position.position.a3,
					       current_joint_position.position.a4,
					       current_joint_position.position.a5,
					       current_joint_position.position.a6,
					       current_joint_position.position.a7);

					group_variable_values2.push_back(current_joint_position.position.a1);
					group_variable_values2.push_back(current_joint_position.position.a2);
					group_variable_values2.push_back(current_joint_position.position.a3);
					group_variable_values2.push_back(current_joint_position.position.a4);
					group_variable_values2.push_back(current_joint_position.position.a5);
					group_variable_values2.push_back(current_joint_position.position.a6);
					group_variable_values2.push_back(current_joint_position.position.a7);
					ROS_INFO("************GET END JOINT PARAM");

					init = true;
				}
			}//end of if(isRobotConnected && !init)

			//Apply the mappinig data to moveit and plan
			else if (isRobotConnected && init) {
				if (!once) {//do this task only one time
                                        group.setJointValueTarget(group_variable_values1);
                                        robot_state::RobotState start_state(group.getJointValueTarget());
                                        group.setStartState(start_state);

                                        group.setJointValueTarget(group_variable_values2);
                                        //success = group.plan(myplan);//original
                                        success = (group.plan(myplan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
					ROS_INFO("*****************Visualizing TbD plan is %s",success?"SUCCEED":"FAILED");
					sleep(5.0);
/*
					group.setJointValueTarget(group_variable_values1);
					ROS_INFO("**********SET START");
					success = group.plan(myplan);
					ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

	//////////////////////////
					ROS_INFO("*****************Start updating CurrentState");
					//variables for updating current state
					robot_state::RobotState start_state(*group.getCurrentState());
					sensor_msgs::JointState joint_state;
					std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points_2;
					trajectory_points_2 = myplan.trajectory_.joint_trajectory.points;
					int index_of_last = trajectory_points_2.size()-1;

					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[0]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[1]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[2]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[3]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[4]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[5]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[6]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[0]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[1]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[2]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[3]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[4]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[5]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[6]);

					jointStateToRobotState (joint_state, start_state);
					group.setStartState(start_state);
					ROS_INFO("*****************Complete updating CurrentState");
	//////////////////////

					group.setJointValueTarget(group_variable_values2);
					ROS_INFO("*****************SET END");
					success = group.plan(myplan);
					ROS_INFO("*****************Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
					sleep(5.0);
*/
					if (success) {
						ROS_INFO("*****************EXECUTE");
						//group.execute(myplan);

						trajectory = myplan.trajectory_;

						std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points;
						trajectory_points = trajectory.joint_trajectory.points;

						std::vector<int>::size_type vectorSize = trajectory_points.size();

						std::vector<double> pos_arr;
						int size;

						//put the all joint values to pos_arr vector
						for (int i=0; i<vectorSize; i++) {
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[0]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[1]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[2]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[3]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[4]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[5]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[6]);
						}
						size = pos_arr.size();

						if (printOutput(&pos_arr[0], size) != 1) {
							printf("Failed to print output!\n");
						} else {
							printf("*****************Print out planned path\n");
						}
					}

					once = true;
				}//end of if (!once)

				loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.

			}//end of if(isRobotConnected && init)
/*
			else {
				//ROS_ERROR("*****************Robot is not connected...");

				if (!once2) {
					ROS_INFO("*****************Reference frame: %s", group.getPlanningFrame().c_str());
					ROS_INFO("*****************Reference frame: %s", group.getEndEffectorLink().c_str());

					moveit_msgs::CollisionObject collision_object;
					collision_object.header.frame_id = group.getPlanningFrame();
					//collision_object.header.frame_id = "my_moving_frame";
					//collision_object.header.frame_id = "wall";
					collision_object.id = "environment";

					//how to know num of meshes???? using topic?
					for (int i=0; i<meshNum; i++) {
						char dir[1024];
						char a[64];

						sprintf(a, "%d", i);
						strcpy (dir, "file:///home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/meshes/environment_mesh_");
						strcat (dir, a);
						strcat (dir, ".off");

						shapes::Mesh* m = shapes::createMeshFromResource(dir);
						shape_msgs::Mesh mesh;
		    				shapes::ShapeMsg mesh_msg;
		    				shapes::constructMsgFromShape(m, mesh_msg);
		    				mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

						geometry_msgs::Pose mesh_pose;
						mesh_pose.position.x = -0.3;
						//mesh_pose.position.y = -0.3;
						mesh_pose.position.z = 0.8;

						//mesh_pose.orientation.w= 0.70710678118;//rotate axis-y 180 degree
						//mesh_pose.orientation.x= 0.70710678118;
						//mesh_pose.orientation.y= 0.0;
						//mesh_pose.orientation.z= 0.0;

						//mesh_pose.orientation.w= 1.0;
						//mesh_pose.orientation.x= 0.0;
						//mesh_pose.orientation.y= 0.0;
						//mesh_pose.orientation.z= 0.0;

						collision_object.meshes.push_back(mesh);
						collision_object.mesh_poses.push_back(mesh_pose);
						collision_object.operation = collision_object.ADD;

					}

					std::vector<moveit_msgs::CollisionObject> collision_objects;
					collision_objects.push_back(collision_object);

					ROS_INFO("*****************Add an object into the world");
					//planning_scene_interface.addCollisionObjects(collision_objects);
					planning_scene_interface.applyCollisionObjects(collision_objects);
					sleep(5.0);

	
					group_variable_values1.push_back(0.563);
					group_variable_values1.push_back(0.396);
					group_variable_values1.push_back(0.129);
					group_variable_values1.push_back(-1.572);
					group_variable_values1.push_back(-0.127);
					group_variable_values1.push_back(1.028);
					group_variable_values1.push_back(0.611);

					group_variable_values2.push_back(-0.394);
					group_variable_values2.push_back(0.502);
					group_variable_values2.push_back(-0.215);
					group_variable_values2.push_back(-1.400);
					group_variable_values2.push_back(-0.128);
					group_variable_values2.push_back(1.161);
					group_variable_values2.push_back(-0.321);

					group.setJointValueTarget(group_variable_values1);
					ROS_INFO("*****************SET START");
					success = group.plan(myplan);
					ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"":"FAILED");


					ROS_INFO("*****************Start updating CurrentState");
					//variables for updating current state
					robot_state::RobotState start_state(*group.getCurrentState());
					sensor_msgs::JointState joint_state;
					std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points_2;
					trajectory_points_2 = myplan.trajectory_.joint_trajectory.points;
					int index_of_last = trajectory_points_2.size()-1;

					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[0]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[1]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[2]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[3]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[4]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[5]);
					joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[6]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[0]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[1]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[2]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[3]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[4]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[5]);
					joint_state.position.push_back(trajectory_points_2[index_of_last].positions[6]);

					jointStateToRobotState (joint_state, start_state);
					group.setStartState(start_state);
					ROS_INFO("*****************Complete updating CurrentState");


					group.setJointValueTarget(group_variable_values2);
					ROS_INFO("*****************SET END");
					success = group.plan(myplan);
					ROS_INFO("*****************Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
					sleep(5.0);

					if (success) {
						ROS_INFO("*****************EXECUTE");
						//group.execute(myplan);

						double planningTime = myplan.planning_time_;
						printf("*****************Planning time : %f\n", planningTime);

						trajectory = myplan.trajectory_;

						std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points;
						trajectory_points = trajectory.joint_trajectory.points;

						std::vector<int>::size_type vectorSize = trajectory_points.size();

						std::vector<double> pos_arr;
						int size;

						//put the all joint values to pos_arr vector
						for (int i=0; i<vectorSize; i++) {

							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[0]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[1]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[2]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[3]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[4]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[5]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[6]);
						}
						size = pos_arr.size();

						if (printOutput(&pos_arr[0], size) != 1) {
							printf("Failed to print output!\n");
						} else {
							printf("*****************Print out planned path\n");
						}
					}
	
					once2 = true;
				}
				ros::Duration(3.0).sleep(); // 5 seconds
			}// end of else
*/
		}//end of if(mode == 2)
	}//end of while

	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;
};

void jointPositionCallback(const iiwa_msgs::JointPosition& jp) {
	if (!isRobotConnected)
		isRobotConnected = !isRobotConnected;
	current_joint_position = jp;

	if(setInput<2)
		setInput += 1;
}

void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps) {
	if (!isRobotConnected)
		isRobotConnected = !isRobotConnected;
	current_cartesian_position = ps;
}

void trajectoryCallback(const moveit_msgs::DisplayTrajectory& dp) {
	if (!rcvtra)
		rcvtra = !rcvtra;
	current_display_trajectory = dp;
}

void poseCallback(const geometry_msgs::Pose& new_pose) {
	eef_pose = new_pose;
	//ROS_INFO("===========================new pose: %f %f %f %f %f %f %f\n", eef_pose.position.x, eef_pose.position.y, eef_pose.position.z, eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w);
}

void ARposeCallback (const geometry_msgs::Pose& ar_pose) {
	if (!rcvarstartpose && !rcvargoalpose) {
		ar_start_pose = ar_pose;
		ROS_INFO("*****************AR start pose is received!************************");
		rcvarstartpose = true;
	}
	else if (rcvarstartpose && !rcvargoalpose)
	{
		ar_goal_pose = ar_pose;
		ROS_INFO("*****************AR goal pose is received!************************");
		rcvargoalpose = true;
	}
}

void moveCommandCallback (const geometry_msgs::Pose& msg) {
	if (!rcvmovecommand) {
		rcvmovecommand = true;
	}
	ROS_INFO("*****************moveCommandCallback is called!************************");
}

int printOutput (double * pos_arr, int size) {
	int i;

	ofp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/planning_output.txt", "w");
	if(!ofp) {
		printf("Can't open output file\n");
		return -1;
	}

	fprintf(ofp, "%d\n", size);

	for (i=0; i<size; i++) {
		fprintf(ofp, "%f ", pos_arr[i]);
	}

	fclose(ofp);
	return 1;
}

int readMeshNum () {
        ifp = fopen("/home/hj/iiwa_stack_ws/src/iiwa_stack_examples/iiwa_tool_examples/src/meshnum.txt", "r");
	if(!ifp) {
		printf("Can't open a file\n");
		return -1;
	}

	fscanf(ifp, "%d", &meshNum);

	fclose(ifp);
	return 1;
}

/*---------------------------Commented Codes---------------------------*/

/*---------------------------load collision object: bunny---------------------------*/
/*
moveit_msgs::CollisionObject collision_object;
collision_object.header.frame_id = group.getPlanningFrame();
collision_object.id = "environment";

//bunny
Eigen::Vector3d scale(0.01, 0.01, 0.01);
shapes::Mesh* m = shapes::createMeshFromResource("file:///home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/other_meshes/bunny.obj", scale);
shape_msgs::Mesh mesh;
shapes::ShapeMsg mesh_msg;
shapes::constructMsgFromShape(m, mesh_msg);
mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
geometry_msgs::Pose mesh_pose;
mesh_pose.position.x = 0.6;
mesh_pose.position.z = 0.0;
//mesh_pose.position.z = 1.0;
mesh_pose.orientation.w= 0.7071068;
mesh_pose.orientation.x= 0.7071068;
mesh_pose.orientation.y= 0.0;
mesh_pose.orientation.z= 0.0;
collision_object.meshes.push_back(mesh);
collision_object.mesh_poses.push_back(mesh_pose);
collision_object.operation = collision_object.ADD;
//
	
std::vector<moveit_msgs::CollisionObject> collision_objects;
collision_objects.push_back(collision_object);

ROS_INFO("*****************Add an object into the world");
//planning_scene_interface.addCollisionObjects(collision_objects);
planning_scene_interface.applyCollisionObjects(collision_objects);
*/

/*------------------------------------------------Tooltip Pose Samples for mode3------------------------------------------------*/
/*
			geometry_msgs::Pose eef_pose_1;
			eef_pose_1.position.x = 0.5;
			eef_pose_1.position.y = 0.0;
			eef_pose_1.position.z = 0.84;
			//y: 180
			eef_pose_1.orientation.x= 0.0;			
			eef_pose_1.orientation.y= 1.0;	
			eef_pose_1.orientation.z= 0.0;
			eef_pose_1.orientation.w= 0.0;
			//y: +180-90 = +90
			//eef_pose_1.orientation.x= 0.0;			
			//eef_pose_1.orientation.y= 0.7071068;	
			//eef_pose_1.orientation.z= 0.0;
			//eef_pose_1.orientation.w= 0.7071068;
			
			geometry_msgs::Pose eef_pose_2;
			eef_pose_2.position.x = 0.6535;
			eef_pose_2.position.y = 0.0;
			eef_pose_2.position.z = 1.58;
			//y: 90
			eef_pose_2.orientation.x= 0.0;			
			eef_pose_2.orientation.y= 0.7071068;	
			eef_pose_2.orientation.z= 0.0;
			eef_pose_2.orientation.w= 0.7071068;
			//y: +90-90 = 0			
			//eef_pose_2.orientation.x= 0.0;			
			//eef_pose_2.orientation.y= 0.0;	
			//eef_pose_2.orientation.z= 0.0;
			//eef_pose_2.orientation.w= 1;
			
			//straight-up
			geometry_msgs::Pose eef_pose_3;
			eef_pose_3.position.x = 0.0;
			eef_pose_3.position.y = 0.0;
			eef_pose_3.position.z = 2.2335;
			//y -90
			eef_pose_3.orientation.x= 0.0;			
			eef_pose_3.orientation.y= -0.7071068;	
			eef_pose_3.orientation.z= 0.0;
			eef_pose_3.orientation.w= 0.7071068;
*/

/*------------------------------------------------Old version of planning code------------------------------------------------*/
/*
				group.setJointValueTarget(ar_start_pose, ee_link);
				//group.setJointValueTarget(eef_pose_2, ee_link);
				ROS_INFO("*****************SET START");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"success":"FAILED");

				ROS_INFO("*****************Start updating CurrentState");
				//variables for updating current state
				robot_state::RobotState start_state(*group.getCurrentState());
				sensor_msgs::JointState joint_state;
				std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points_2;
				trajectory_points_2 = myplan.trajectory_.joint_trajectory.points;
				int index_of_last = trajectory_points_2.size()-1;

				joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[0]);
				joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[1]);
				joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[2]);
				joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[3]);
				joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[4]);
				joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[5]);
				joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[6]);
				joint_state.position.push_back(trajectory_points_2[index_of_last].positions[0]);
				joint_state.position.push_back(trajectory_points_2[index_of_last].positions[1]);
				joint_state.position.push_back(trajectory_points_2[index_of_last].positions[2]);
				joint_state.position.push_back(trajectory_points_2[index_of_last].positions[3]);
				joint_state.position.push_back(trajectory_points_2[index_of_last].positions[4]);
				joint_state.position.push_back(trajectory_points_2[index_of_last].positions[5]);
				joint_state.position.push_back(trajectory_points_2[index_of_last].positions[6]);

				jointStateToRobotState (joint_state, start_state);
				group.setStartState(start_state);
				ROS_INFO("*****************Complete updating CurrentState");

				group.setJointValueTarget(ar_goal_pose, ee_link);
				//group.setJointValueTarget(eef_pose_3, ee_link);
				ROS_INFO("*****************SET END");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
				sleep(5.0);
*/

/*------------------------------------------------Studies for using group.setJointValueTarget( , );------------------------------------------------*/
/*		
//sample1
			group.setJointValueTarget(eef_pose_2, ee_link);
			ROS_INFO("*****************SET START");
			success = group.plan(myplan);
			ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"success":"FAILED");

			ROS_INFO("*****************Start updating CurrentState");
			//variables for updating current state
			robot_state::RobotState start_state(*group.getCurrentState());
			sensor_msgs::JointState joint_state;
			std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points_2;
			trajectory_points_2 = myplan.trajectory_.joint_trajectory.points;
			int index_of_last = trajectory_points_2.size()-1;

			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[0]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[1]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[2]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[3]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[4]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[5]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[6]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[0]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[1]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[2]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[3]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[4]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[5]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[6]);

			jointStateToRobotState (joint_state, start_state);
			group.setStartState(start_state);
			ROS_INFO("*****************Complete updating CurrentState");

			group.setJointValueTarget(eef_pose_2, ee_link);
			ROS_INFO("*****************SET START");
			success = group.plan(myplan);
			ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"success":"FAILED");
*/
		
/*sample 2
//-------------------------------------------------------------------------------------------------------
			robot_state::RobotStatePtr robot_k_state;
			robot_k_state.reset(new robot_state::RobotState(group.getRobotModel()));
			robot_k_state->setToDefaultValues();

			bool found_ik = robot_k_state->setFromIK((group.getRobotModel())->getJointModelGroup(movegroup_name), eef_pose_2, ee_link, 0, 0);
			//bool found_ik = robot_k_state->setFromIK((group.getRobotModel())->getJointModelGroup(movegroup_name), eef_pose_2, ee_link, 10, 0.1);
			ROS_INFO("*****************find IK %s", found_ik?"OK":"FAILED");
			if(found_ik) {
				//robot_k_state->printStateInfo(std::cout);
				//group.setStartState(*robot_k_state);
				group.setJointValueTarget(*robot_k_state);
				ROS_INFO("*****************SET START");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"success":"FAILED");
			}
//-------------------------------------------------------------------------------------------------------
			ROS_INFO("*****************Start updating CurrentState");
			//variables for updating current state
			robot_state::RobotState start_state(*group.getCurrentState());
			sensor_msgs::JointState joint_state;
			std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points_2;
			trajectory_points_2 = myplan.trajectory_.joint_trajectory.points;
			int index_of_last = trajectory_points_2.size()-1;

			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[0]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[1]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[2]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[3]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[4]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[5]);
			joint_state.name.push_back(myplan.trajectory_.joint_trajectory.joint_names[6]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[0]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[1]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[2]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[3]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[4]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[5]);
			joint_state.position.push_back(trajectory_points_2[index_of_last].positions[6]);

			jointStateToRobotState (joint_state, start_state);
			group.setStartState(start_state);
			ROS_INFO("*****************Complete updating CurrentState");
//-------------------------------------------------------------------------------------------------------
			found_ik = robot_k_state->setFromIK((group.getRobotModel())->getJointModelGroup(movegroup_name), eef_pose_2, ee_link, 10, 0.1);
			ROS_INFO("*****************find IK %s", found_ik?"OK":"FAILED");
			if(found_ik) {
				
				//group.setStartState(*robot_k_state);
				group.setJointValueTarget(*robot_k_state);
				ROS_INFO("*****************SET END");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 2 (pose goal) %s",success?"success":"FAILED");
			}
//-------------------------------------------------------------------------------------------------------	
*/

			//robot_state::RobotState start_state(*group.getCurrentState());
			//group.setStartState(start_state);
			//group.setStartState(*robot_k_state);

/*
			if (success) {
				ROS_INFO("*****************EXECUTE");
				//group.execute(myplan);

				double planningTime = myplan.planning_time_;
				printf("*****************Planning time : %f\n", planningTime);

				trajectory = myplan.trajectory_;

				std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points;
				trajectory_points = trajectory.joint_trajectory.points;

				std::vector<int>::size_type vectorSize = trajectory_points.size();
				std::vector<double> pos_arr;
				int size;

				//put the all joint values to pos_arr vector
				for (int i=0; i<vectorSize; i++) {

					pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[0]);
					pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[1]);
					pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[2]);
					pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[3]);
					pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[4]);
					pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[5]);
					pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[6]);
				}
				size = pos_arr.size();

				if (printOutput(&pos_arr[0], size) != 1) {
					printf("Failed to print output!\n");
				} else {
					printf("*****************Print out planned path\n");
				}
			}
*/

/*------------------------------------------------Old version of planning code: mode2------------------------------------------------*/
/*
				group.setJointValueTarget(group_variable_values1);
				ROS_INFO("**********SET START");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

				group.setJointValueTarget(group_variable_values2);
				ROS_INFO("************SET END");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 2 (pose goal) %s",success?"":"FAILED");

				if (success) {
					ROS_INFO("**********EXECUTE");
					group.execute(myplan);

					if (!once) {//do this task only one time
						trajectory = myplan.trajectory_;

						std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points;
						trajectory_points = trajectory.joint_trajectory.points;

						std::vector<int>::size_type vectorSize = trajectory_points.size();

						std::vector<double> pos_arr;
						int size;

						//put the all joint values to pos_arr vector
						for (int i=0; i<vectorSize; i++) {
							////Check the size of the positions array
							//std::vector<double> pos = trajectory.joint_trajectory.points[i].positions;
							//std::vector<int>::size_type vectorSize2 = pos.size();
							//ROS_INFO("The size of pos = %i", vectorSize2);//7
							//ROS_INFO("%f", trajectory.joint_trajectory.points[i].positions[0]);

							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[0]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[1]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[2]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[3]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[4]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[5]);
							pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[6]);
						}
						size = pos_arr.size();

						if (printOutput(&pos_arr[0], size) != 1) {
							printf("Failed to print output!\n");
						} else {
							printf("Print out planned path\n");
						}

						once = true;
					}//end of if (!once)
				}//end of if (success)
	*/
