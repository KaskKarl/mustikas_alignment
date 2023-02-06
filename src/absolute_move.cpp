#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

bool trigger = false;
geometry_msgs::Pose goal;

void callback(geometry_msgs::Pose data)
{
	goal.position.x = data.position.x;
	goal.position.y = data.position.y;
	goal.position.z = data.position.z;
	goal.orientation.x = data.orientation.x;
	goal.orientation.y = data.orientation.y;
	goal.orientation.z = data.orientation.z;
	goal.orientation.w = data.orientation.w;
	
	trigger = true;

}

int main(int argc, char** argv)
{
	//ROS setup
	ros::init(argc, argv, "absolute_move");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	ros::Subscriber sub = node_handle.subscribe("robot_coords", 1, callback);
	
	//interface setup with name xarm6
	moveit::planning_interface::MoveGroupInterface move_group("xarm6");
	
	//creating an empty plan
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
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
			move_group.execute(my_plan);
			
			ROS_INFO("[incemental_move/pose_goal] Execution complete");
			
			trigger = false;
		}
	}
	
	ros::shutdown();
	return 0;


}
