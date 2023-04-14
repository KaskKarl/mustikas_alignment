#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>




bool trigger = false;

bool moveToPoint(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{	
	trigger = req.data;
		
	res.success = true;
	res.message = "Message received!";

	
	return true;
}


int main(int argc, char** argv)
{
	//ROS setup
	ros::init(argc, argv, "ready_move");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// Interface setup with name xarm6
	moveit::planning_interface::MoveGroupInterface move_group("xarm6");

	// Creating an empty plan
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	// Creating a service server
	ros::ServiceServer service = node_handle.advertiseService("ready_move", moveToPoint);
	
	while (ros::ok())
	{
		if (trigger)
		{
			const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("xarm6");
			moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
			
			//Setting the goal joint states
			std::vector<double> joint_positions;
			
			current_state->copyJointGroupPositions(joint_model_group, joint_positions);
			
			joint_positions[0] = 0.0;
			joint_positions[1] = -1.1868;
			joint_positions[2] = 0.0;
			joint_positions[3] = 0.0;
			joint_positions[4] = -0.2094;
			joint_positions[5] = 0.0;
						
			//Setting target pose equal to current
			move_group.setJointValueTarget(joint_positions);
			
			bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			
			ROS_INFO_NAMED("READY MOVE", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
			
			//Executing plan
			ros::Duration(0.5).sleep();
			move_group.execute(my_plan);
			
			ROS_INFO("[ready_move] Execution complete");
			trigger = false;
		
		}
	
	}
	
	ros::shutdown();
	return 0;


}



