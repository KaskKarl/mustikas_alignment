#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <mustikas_alignment/Control.h>
#include <mustikas_alignment/MoveToPoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <stdio.h>

ros::Publisher pub;
ros::Publisher drive_pub;
geometry_msgs::Pose goal;
ros::Time msg_time;
int time_l;

std_msgs::Bool stop_msg;
std_msgs::String drive_msg;


// Variable to store the functionality activated from the remote
std::string trigger_name = "STANDBY";
bool msg_trigger = false;


void cb_robot_coords(geometry_msgs::Pose msg)
{
	goal = msg;
	msg_trigger = true;
	msg_time = ros::Time::now();
}

bool control(mustikas_alignment::Control::Request &req, mustikas_alignment::Control::Response &res)
{
	if (req.data == "GOAL")
	{
		std::cout << "Control has been turned to GOAL!" << std::endl;
		res.message = "Control has been turned to GOAL!";
		res.success = true;
		trigger_name = "GOAL";
	}
	else if (req.data == "READY") 
	{
		std::cout << "Control has been turned to READY!" << std::endl;
		res.message = "Control has been turned to READY!";
		res.success = true;
		trigger_name = "READY";
	}
	else if (req.data == "AUTO") 
	{
		std::cout << "Control has been turned to AUTO!" << std::endl;
		res.message = "Control has been turned to AUTO!";
		res.success = true;
		trigger_name = "AUTO";
	}
	else if (req.data == "STOP")
	{
		std::cout << "Execution STOPPED!" << std::endl;
		res.message = "Execution STOPPED!";
		res.success = true;
		trigger_name = "STOP";
		pub.publish(stop_msg);
	}
	else 
	{
		std::cout << "Unknown command!" << std::endl;
		res.message = "Unknown command!";
		res.success = false;
		trigger_name = "STANDBY";
	}


return true;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "xarm_control_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	nh.getParam("/mustikas/msg_time_limit", time_l);
	ros::Duration time_limit(time_l,0);
	
	// Interface setup with name xarm6
	moveit::planning_interface::MoveGroupInterface move_group("xarm6");

	// Creating an empty plan
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("xarm6");
    
    pub = nh.advertise<std_msgs::Bool>("/stop_cmd", 1);
    drive_pub = nh.advertise<std_msgs::String>("/drive_commands", 1);
    stop_msg.data = true;
    drive_msg.data = "go";
	
	// Subscriber to the plant coordinates topic
	ros::Subscriber coord_sub = nh.subscribe("mustikas_goal", 1, cb_robot_coords);
	
	// Service server for the remotecontrol to connect to
	ros::ServiceServer service = nh.advertiseService("xarm_control", control);

	
	while (ros::ok())
	{
		if (trigger_name == "READY")
		{
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
			
			ROS_INFO("[xarm_control_node/ready] Execution complete");
			trigger_name = "STANDBY";
		}
		if ((ros::Time::now() - msg_time) <= time_limit)
		{
			if (trigger_name == "GOAL")
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
					ROS_INFO("[xarm_control_node/goal] Planning OK. Proceeding...");
				}
				else
				{
					ROS_WARN("[xarm_control_node/goal] Planning failed. Shutting down");
					
				}
				
				//Executing plan
				ros::Duration(0.5).sleep();
				move_group.execute(my_plan);
				
				ROS_INFO("[xarm_control_node/goal] Execution complete");
				trigger_name = "STANDBY";
				
			}
			else if (trigger_name == "AUTO" and msg_trigger)
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
					ROS_INFO("[xarm_control_node/auto_to_goal] Planning OK. Proceeding...");
				}
				else
				{
					ROS_WARN("[xarm_control_node/auto_to_goal] Planning failed. Shutting down");
					
				}
				
				//Executing plan
				ros::Duration(0.5).sleep();
				move_group.execute(my_plan);
				
				ROS_INFO("[xarm_control_node/auto_to_goal] Execution complete");
				
				ros::Duration(2).sleep();
				
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
				
				success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				
				ROS_INFO_NAMED("READY MOVE", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
				
				//Executing plan
				ros::Duration(0.5).sleep();
				move_group.execute(my_plan);
				
				ROS_INFO("[xarm_control_node/auto_to_ready] Execution complete");
				
				if (success)
				{
					drive_pub.publish(drive_msg);
				}
				
				msg_trigger = false;
			}
			else
			{
				// Do nothing
			}
			}
		
	}
	
	ros::shutdown();
	return 0;
}
