#include "ros/ros.h"
//#include <move_group.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/Pose.h>


using namespace std;

std::string joint_position_topic, cartesian_position_topic, movegroup_name, ee_link;
double ros_rate = 0.1;

moveit_msgs::RobotTrajectory trajectory;//#include <RobotTrajectory.h>




int main (int argc, char **argv) {
	ros::init(argc, argv, "CommandRobot");
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
	
	//Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;//#include <DisplayTrajectory.h>

	geometry_msgs::Pose target_pose1;//#include <Pose.h>
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.y = -0.7;
	target_pose1.position.z = 1.0;
	group.setPoseTarget(target_pose1);	
	
	moveit::planning_interface::MoveGroup::Plan my_plan;//#include <move_group.h>
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);

	if (1)
	{
  		ROS_INFO("Visualizing plan 1 (again)");
  		display_trajectory.trajectory_start = my_plan.start_state_;
  		display_trajectory.trajectory.push_back(my_plan.trajectory_);
  		display_publisher.publish(display_trajectory);

  		/* Sleep to give Rviz time to visualize the plan. */
  		sleep(5.0);
	}

	












	spinner.stop();
	return 0;
}
