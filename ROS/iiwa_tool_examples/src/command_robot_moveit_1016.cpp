#include "ros/ros.h"
//#include <move_group.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
//#include <geometry_msgs/Pose.h>


using namespace std;

std::string joint_position_topic, cartesian_position_topic, movegroup_name, ee_link;
double ros_rate = 0.1;
bool isRobotConnected = false;
moveit_msgs::RobotTrajectory trajectory;//#include <RobotTrajectory.h>
//moveit_msgs::RobotState state;

//for print a path in form of .txt
FILE *ofp;


int printOutput (double * pos_arr, int size);

int main (int argc, char **argv) {
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

//---------------------------------------------------------------------------------------------------------------------------------------------
	// Create MoveGroup
	moveit::planning_interface::MoveGroup group(movegroup_name);//#include <move_group.h>
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;//#include <planning_scene_interface.h>
	
/*
	//Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;//#include <DisplayTrajectory.h>
*/
	// Configure planner
  	group.setPlanningTime(0.5);
  	group.setPlannerId("RRTConnectkConfigDefault");
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
	moveit::planning_interface::MoveGroup::Plan myplan;//#include <move_group.h>
	bool success = false;
	std::vector<double> pos_arr;
	int pointNum;

	if(isRobotConnected)
	{
	}
	else {  
		group_variable_values1.push_back(-0.400078);
		group_variable_values1.push_back(0.602474);
		group_variable_values1.push_back(-0.126794);
		group_variable_values1.push_back(-1.409570);
		group_variable_values1.push_back(-0.150624);
		group_variable_values1.push_back(0.140248);
		group_variable_values1.push_back(-0.158617);

		group_variable_values2.push_back(0.373684);
		group_variable_values2.push_back(0.654827);
		group_variable_values2.push_back(0.132620);
		group_variable_values2.push_back(-1.439332);
		group_variable_values2.push_back(0.025667);
		group_variable_values2.push_back(0.672484);
		group_variable_values2.push_back(0.445724);

		//group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values1);
		//group_variable_values1[0] = -0.400080;
		//group.setStartState(group_variable_values1);
		
		

		group.setJointValueTarget(group_variable_values1);
		ROS_INFO("**********SET START");
		success = group.plan(myplan);
		ROS_INFO("*****************Visualizing plan 0 (joint space goal) %s",success?"":"FAILED");
/*
		ROS_INFO("--------------------------------------------------------------------------------------");


		ROS_INFO("%lf", myplan.planning_time_);

		for (int i=0; i<7; i++) {
			ROS_INFO("%s", myplan.start_state_.joint_state.name[i]);
			ROS_INFO("%lf", myplan.start_state_.joint_state.position[i]);
		}
*/
/*
		for (int i=0; i<7; i++) {
			ROS_INFO("%s", myplan.start_state_.multi_dof_joint_state.joint_names[i]);

			ROS_INFO(myplan.start_state_.multi_dof_joint_state.transforms[i].translation.x);
			ROS_INFO(myplan.start_state_.multi_dof_joint_state.transforms[i].translation.y);
			ROS_INFO(myplan.start_state_.multi_dof_joint_state.transforms[i].translation.z);
			ROS_INFO(myplan.start_state_.multi_dof_joint_state.transforms[i].rotation.x);
			ROS_INFO(myplan.start_state_.multi_dof_joint_state.transforms[i].rotation.y);
			ROS_INFO(myplan.start_state_.multi_dof_joint_state.transforms[i].rotation.z);
			ROS_INFO(myplan.start_state_.multi_dof_joint_state.transforms[i].rotation.w);

		}

		ROS_INFO("--------------------------------------------------------------------------------------");
*/



		//group.setStartStateToCurrentState();

		//myplan.start_state_.joint_state.position.push_back(group_variable_values1);
		myplan.start_state_.joint_state.position[0] = group_variable_values1[0];
		myplan.start_state_.joint_state.position[1] = group_variable_values1[1];
		myplan.start_state_.joint_state.position[2] = group_variable_values1[2];
		myplan.start_state_.joint_state.position[3] = group_variable_values1[3];
		myplan.start_state_.joint_state.position[4] = group_variable_values1[4];
		myplan.start_state_.joint_state.position[5] = group_variable_values1[5];
		myplan.start_state_.joint_state.position[6] = group_variable_values1[6];



		group.setJointValueTarget(group_variable_values2);
		ROS_INFO("************SET END");
        	success = group.plan(myplan); 
        	ROS_INFO("*****************Visualizing plan 1 (joint space goal) %s",success?"":"FAILED");

	


		
/*
		group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values1);
		group_variable_values1[0] = -0.400080;
		group.setJointValueTarget(group_variable_values1);
		success = group.plan(myplan);
		ROS_INFO("*****************Visualizing plan 1 (joint space goal) %s",success?"":"FAILED");
		sleep(5.0);
*/


		if (success) {
	  		group.execute(myplan);
			ROS_INFO("**********EXECUTE");
			
			trajectory = myplan.trajectory_;
		  	pointNum = trajectory.joint_trajectory.points.size();
		  
		  	//put the all joint values to pos_arr vector
		  	for (int i=0; i<pointNum; i++) {
		    		pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[0]);
		    		pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[1]);
		    		pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[2]);
		    		pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[3]);
		    		pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[4]);
		    		pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[5]);
		    		pos_arr.push_back(trajectory.joint_trajectory.points[i].positions[6]);
		  	}

		  	if (printOutput(&pos_arr[0], pos_arr.size()) != 1) {
				  printf("Failed to print output!\n");
			} else {
		    		printf("Print out planned path\n");
		  	}
		}
    	}
	spinner.stop();
	return 0;
}

int printOutput (double * pos_arr, int size) {
	int i;

	ofp = fopen("/home/glab/hj/src/iiwa_tool-master/iiwa_tool_examples/src/planning_output_test.txt", "w");
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
