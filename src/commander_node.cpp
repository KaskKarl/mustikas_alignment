#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <cmath>
#include <stdio.h>

ros::Publisher pub;
ros::Publisher drive_pub;
geometry_msgs::PoseArray mustikas_array;

// Variable to keep track of the number of plants fertilized
int mustikas_counter = 0;
ros::Time fert_time(0.001);
ros::Duration igno_time(5,0);

std_msgs::String stop;


void cb_array(const geometry_msgs::PoseArray objects)
{
	mustikas_array = objects;
}

/*
void cb_completed()
{
	//1. Mark fertilization to complete.
}
*/
int main (int argc, char** argv)
{
	float fert_area_center_x = 0.5; //distance from robot
	float fert_area_center_y = 0.0; //offset from center
	float fert_area_width = 0.5;
	float fert_area_length = 0.2;
	float mount_angle = 1.38754;
	
	stop.data = "stop";
	
	ros::init(argc, argv, "mustikas_commander_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	pub = nh.advertise<geometry_msgs::Pose>("/mustikas_goal", 1);
	drive_pub = nh.advertise<std_msgs::String>("/drive_commands", 1);
	
	ros::Subscriber sub1 = nh.subscribe("/robot_coords", 1 ,cb_array);
	//ros::Subscriber sub2 = nh.subscribe("/mustikas_completed", 1, cb_completed);
	
	
	
	while (ros::ok())
	{
		
		for (auto i : mustikas_array.poses)
		{
			float projected_x = sqrt(i.position.x*i.position.x + i.position.z*i.position.z) * cos(mount_angle - atan(i.position.z/i.position.x));
			
			if ((projected_x < (fert_area_center_x + (fert_area_length/2))) and (projected_x > (fert_area_center_x - (fert_area_length/2))) and (i.position.y < (fert_area_center_y + (fert_area_width/2))) and (i.position.y > (fert_area_center_y - (fert_area_width/2))))
			{
				std::cout << "plant in area" << std::endl;
				if ((ros::Time::now() - fert_time) >= igno_time)
				{
					drive_pub.publish(stop);
					ros::Duration(2).sleep();
					pub.publish(i);
					fert_time = ros::Time::now();
				}
				
			}
		}
		
	
	
	}
	
	
	ros::shutdown();
	return 0;
}
