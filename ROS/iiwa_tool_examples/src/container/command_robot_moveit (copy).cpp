//Joint angle target input

#include <stdio.h>
#include <vector>

#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>

#include </opt/ros/indigo/include/moveit/planning_scene_interface/planning_scene_interface.h>
#include </opt/ros/indigo/include/moveit_msgs/DisplayRobotState.h>
#include </opt/ros/indigo/include/moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/shape_operations.h>

//#include "numofmesh.h"

using namespace std;

//for planning
std::string joint_position_topic, cartesian_position_topic, movegroup_name, ee_link;
iiwa_msgs::JointPosition current_joint_position;
geometry_msgs::PoseStamped current_cartesian_position;
moveit_msgs::RobotTrajectory trajectory;
//robot_state::RobotState start_state;


//not used
geometry_msgs::PoseStamped command_cartesian_position;
geometry_msgs::Pose end, target_pose1;


double ros_rate = 0.1;
bool isRobotConnected = false;
int setInput = 0;

//for check the num of printing
bool printStart = false;
bool once = false;

//for print a path in form of .txt
FILE *ofp;
FILE *ifp;

//for !isRobotConnected
bool once2 = false;


void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps);
int printOutput (double * pos_arr, int size);
int printState (double * start, double * goal);
int readState (double * start, double * goal);

int main (int argc, char **argv) {
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
	nh.param<std::string>("ee_link", ee_link, "tool_link_ee");

	// Dynamic parameter to choose the rate at wich this node should run
	nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

	// Subscribers and publishers
	ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
	ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
	//ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);//maybe not used
	//ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 100);



	// Create MoveGroup
	move_group_interface::MoveGroup group(movegroup_name);

	moveit::planning_interface::MoveGroup::Plan myplan;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Configure planner
	//group.setPlanningTime(0.5);
	group.setPlanningTime(5.0);
	//group.setPlannerId("RRTConnectkConfigDefault");
	group.setPlannerId("RRTstarkConfigDefault");
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

	bool init = false;
	std::vector<double> group_variable_values1, group_variable_values2;
	bool success = false;
	
	while (ros::ok()) {
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
	
				//group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values1);

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

		else if (isRobotConnected && init) {
			

			//ROS_INFO("*****************number of meshes: %d", meshNum);

			

			if (!once) {//do this task only one time
				if (printState (&group_variable_values1[0], &group_variable_values2[0]) != 1) {
					printf("Failed to print output!\n");
				} else {
					printf("*****************Print out state\n");
				}


				moveit_msgs::CollisionObject collision_object;
				collision_object.header.frame_id = group.getPlanningFrame();
				collision_object.id = "environment";

				//how to know num of meshes???? using topic?
				for (int i=0; i<73; i++) {
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
					//mesh_pose.position.x = -0.3;
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

				group.setJointValueTarget(group_variable_values1);  
				ROS_INFO("**********SET START");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

/*
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
*/

				group.setJointValueTarget(group_variable_values2);
				ROS_INFO("*****************SET END");
				success = group.plan(myplan);
				ROS_INFO("*****************Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
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

				once = true;
			}//end of if (!once)

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
			loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.

		}//end of if(isRobotConnected && init)
		else {
/*
			//ROS_ERROR("*****************Robot is not connected...");

			if (!once2) {
				ROS_INFO("*****************Reference frame: %s", group.getPlanningFrame().c_str());
				ROS_INFO("*****************Reference frame: %s", group.getEndEffectorLink().c_str());

				moveit_msgs::CollisionObject collision_object;
				collision_object.header.frame_id = group.getPlanningFrame();
				//collision_object.header.frame_id = "my_moving_frame";
				//collision_object.header.frame_id = "wall";
				collision_object.id = "environment";
*/

/*
				//how to know num of meshes???? using topic?
				for (int i=0; i<73; i++) {
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
*/
/*
				//
				Eigen::Vector3d scale(0.1, 0.1, 0.1);
				shapes::Mesh* m = shapes::createMeshFromResource("file:///home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/other_meshes/bunny.obj", scale); 
				shape_msgs::Mesh mesh;
	    			shapes::ShapeMsg mesh_msg;
	    			shapes::constructMsgFromShape(m, mesh_msg);
	    			mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
				geometry_msgs::Pose mesh_pose;
				mesh_pose.position.x = 0.6;
				//mesh_pose.position.y = -0.3;
				mesh_pose.position.z = 1.0;



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
				sleep(5.0);


				double start[7], goal[7];
				if (readState (start, goal) != 1) {
					printf("Failed to read!\n");
				} else {
					printf("Succeded to read input\n");
				}


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
*/
			ros::Duration(3.0).sleep(); // 5 seconds
		}// end of else

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

int printState (double * start, double * goal) {
	int i;

	ofp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/state.txt", "w");
	if(!ofp) {
		printf("Can't open output file\n");
		return -1;
	}
	
	for (i=0; i<7; i++) {
		fprintf(ofp, "%f ", start[i]);
	}

	for (i=0; i<7; i++) {
		fprintf(ofp, "%f ", goal[i]);
	}

	fclose(ofp);
	return 1;
}

int readState (double * start, double * goal) {
	int i;

	ifp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/state.txt", "r");
	if(!ifp) {
		printf("Can't open a file\n");
		return -1;
	}

	for (i=0; i<7; i++) {
		fscanf(ifp, "%lf", &start[i]);
	}

	for (i=0; i<7; i++) {
		fscanf(ifp, "%lf", &goal[i]);
	}

	fclose(ifp);
	return 1;
}
