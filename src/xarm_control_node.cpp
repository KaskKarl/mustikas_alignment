#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <mustikas_alignment/Control.h>
#include <mustikas_alignment/MoveToPoint.h>

#include <stdio.h>

geometry_msgs::Pose goal;
mustikas_alignment::MoveToPoint srv_goal;
std_srvs::SetBool srv_ready;




// Variable to store the functionality activated from the remote
std::string trigger_name = "STANDBY";



void cb_robot_coords(geometry_msgs::Pose msg)
{
	srv_goal.request.pose = msg;
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
	
	// Subscriber to the plant coordinates topic
	ros::Subscriber coord_sub = nh.subscribe("/robot_coords", 1, cb_robot_coords);
	
	// Service server for the remotecontrol to connect to
	ros::ServiceServer service = nh.advertiseService("xarm_control", control);
	
	// Service client for moving to transport pose
	
	// Service client for moving to ready pose
	ros::ServiceClient ready_client = nh.serviceClient<std_srvs::SetBool>("ready_move");
	
	// Service client for moving to the plant
	ros::ServiceClient goal_client = nh.serviceClient<mustikas_alignment::MoveToPoint>("absolute_move");
	
	
	
	while (ros::ok())
	{
		if (trigger_name == "GOAL")
		{
			// Call goal service
			goal_client.call(srv_goal);
			trigger_name = "STANDBY";
		}
		else if (trigger_name == "READY")
		{
			// Call ready service
			srv_ready.request.data = true;
			ready_client.call(srv_ready);
			trigger_name = "STANBY";
		}
		else
		{
			// Do nothing
		}
	
	}
	
	ros::shutdown();
	return 0;
}
