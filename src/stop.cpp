#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Bool.h>

bool trigger = false;

void cb(const std_msgs::Bool msg)
{
	trigger = true;
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "xarm_stop_node");
	ros::NodeHandle nh;
	moveit::planning_interface::MoveGroupInterface move_group("xarm6");
	
	ros::Subscriber sub = nh.subscribe("/stop_cmd", 10, cb);
	
	if (trigger == true)
	{
		move_group.stop();
		trigger = false;
	}
	
    ros::spin();

}
