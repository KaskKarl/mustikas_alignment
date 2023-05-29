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

bool fert_trigger = false;

float fert_area_center_x; //distance from robot
float fert_area_center_y; //offset from center
float fert_area_width;
float fert_area_length;
float mount_angle;
float offset_x;
int igno_t;

ros::Duration igno_time(5,0);

std_msgs::String stop;
std_msgs::String drive;

void cb_array(const geometry_msgs::PoseArray objects)
{
	
	if ((ros::Time::now() - fert_time) >= igno_time)
	{
		mustikas_array = objects;
		fert_trigger = true;
	}
}

/*
void cb_completed()
{
	//1. Mark fertilization to complete.
}
*/
int main (int argc, char** argv)
{
	

	
	stop.data = "stop";
	drive.data = "go";
	
	ros::init(argc, argv, "mustikas_commander_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	nh.getParam("/mustikas/fert_area/center_x", fert_area_center_x);
	nh.getParam("/mustikas/fert_area/center_y", fert_area_center_y);
	nh.getParam("/mustikas/fert_area/width", fert_area_width);
	nh.getParam("/mustikas/fert_area/length", fert_area_length);
	nh.getParam("/mustikas/camera/angle", mount_angle);
	nh.getParam("/mustikas/camera/offset_x", offset_x);
	nh.getParam("/mustikas/igno_time", igno_t);
	
	pub = nh.advertise<geometry_msgs::Pose>("/mustikas_goal", 1);
	drive_pub = nh.advertise<std_msgs::String>("/drive_commands", 1);
	
	ros::Subscriber sub1 = nh.subscribe("/robot_coords", 1 ,cb_array);
	//ros::Subscriber sub2 = nh.subscribe("/mustikas_completed", 1, cb_completed);
	
	
	
	while (ros::ok())
	{
		
		for (auto i : mustikas_array.poses)
		{
			float projected_x = sqrt(i.position.x*i.position.x + i.position.z*i.position.z) * cos(mount_angle - atan(i.position.z/i.position.x)) - offset_x;
			std::cout << "x: " << projected_x << std::endl;
			
			if ((projected_x < (fert_area_center_x + (fert_area_length/2))) and (projected_x > (fert_area_center_x - (fert_area_length/2))) and (i.position.y < (fert_area_center_y + (fert_area_width/2))) and (i.position.y > (fert_area_center_y - (fert_area_width/2))))
			{

				if (fert_trigger)
				{
					drive_pub.publish(stop);
					ros::Duration(2).sleep();
					pub.publish(i);
					fert_time = ros::Time::now();
					fert_trigger = false;
				}
				else
				{
					drive_pub.publish(drive);
				}
			}
		}
		
	
	
	}
	
	
	ros::shutdown();
	return 0;
}
