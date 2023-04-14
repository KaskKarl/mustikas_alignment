#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mustikas_alignment/MoveToPoint.h>



geometry_msgs::Pose goal;
bool trigger = false;

bool moveToPoint(mustikas_alignment::MoveToPoint::Request &req, mustikas_alignment::MoveToPoint::Response &res)
{
	goal = req.pose;
		
	res.success = true;
	res.message = "Message received!";
	
	trigger = true;
	
	return true;
}


int main(int argc, char** argv)
{
	//ROS setup
	ros::init(argc, argv, "absolute_move");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// Interface setup with name xarm6
	moveit::planning_interface::MoveGroupInterface move_group("xarm6");

	// Creating an empty plan
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	// Creating a service server
	ros::ServiceServer service = node_handle.advertiseService("absolute_move", moveToPoint);
	
	while (ros::ok())
	{
		if (trigger)
		{
			//getting the current pose
			geometry_msgs::PoseStamped current_pose;
			current_pose = move_group.getCurrentPose();
			
			//Setting target pose equal to current
			geometry_msgs::Pose target_pose = current_pose.pose;
			
			//Modifying the target pose
			target_pose.position.x = goal.position.x;
			target_pose.position.y = goal.position.y;
			target_pose.position.z = goal.position.z;
			
			
			target_pose.orientation.x = goal.orientation.x;
			target_pose.orientation.y = goal.orientation.y;
			target_pose.orientation.z = goal.orientation.z;
			target_pose.orientation.w = goal.orientation.w;
			
			// Cartesian move planning

			/*
			std::vector<geometry_msgs::Pose> waypoints;
			waypoints.push_back(target_pose);
			
			moveit_msgs::RobotTrajectory trajectory;
			const double jump_threshold = 0.0;
			const double eef_step = 0.01;
			double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			
			*/
			
			// Regular planning
			move_group.setPoseTarget(target_pose);
			
			
			moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
			
			//checking if planning was successful
			if (success) 
			{
				ROS_INFO("[incremental_move/pose_goal] Planning OK. Proceeding...");
			}
			else
			{
				ROS_WARN("[incremental_move/pose_goal] Planning failed. Shutting down");
				
			}
			
			//Executing plan
			ros::Duration(0.5).sleep();
			//move_group.execute(trajectory);
			move_group.execute(my_plan);
			
			ROS_INFO("[absolute_move/cartesian_path] Execution complete");
			trigger = false;
		
		}
	
	}
	
	ros::shutdown();
	return 0;


}



