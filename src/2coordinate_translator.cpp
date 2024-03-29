#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <mustikas_alignment/PointArray.h>

//variables to define the camera coordinate origin relative to the robot's origin
float camera_offset_x;
float camera_offset_y; //positive value means camera is to the left of the arm
float camera_offset_z;

float cam_angle; 
float xarm_angle;

//additional positioning offsets. Used when collision with the object needs to be avoided
float offset_x;
float offset_y;
float offset_z;

// Angle of the tool. Only 1 is used at the moment to align the tool to the ground

double roll;
double pitch;
double yaw;


geometry_msgs::Point old_coords;
geometry_msgs::Pose new_coord;
geometry_msgs::Pose between_coord;
geometry_msgs::PoseArray new_coords;

// Variables for holding the coordinate values after adding the offsets and before the rotation
float x, z;

ros::Publisher pub;



void cb(const mustikas_alignment::PointArray::ConstPtr& msg)
{
	int counter = 0;
	
	for(auto i : msg->points) {
		old_coords = i;
		//Checking if the object has x coordinate. Objects that are too far or close return an Nan and are therefore ignored
		if (!std::isnan(old_coords.x))
		{
			new_coord.position.y = (-old_coords.x) + camera_offset_y + offset_y;
			z = -old_coords.y;
			x = old_coords.z;
			
			//Using a rotation matrix to rotate the x and z coordinates parallel to the fielrobot coordinates
			between_coord.position.x = x*cos(-cam_angle) - z*sin(-cam_angle);
			between_coord.position.z = x*sin(-cam_angle) + z*cos(-cam_angle);
			
			//Adding the camera and goal offsets
			between_coord.position.x = between_coord.position.x + camera_offset_x + offset_x;
			between_coord.position.z = between_coord.position.z + camera_offset_z + offset_z;
		
			//Using a rotation matrix to rotate the x and z coordinates to the xarm coordinates
			new_coord.position.x = between_coord.position.x*cos(xarm_angle) - between_coord.position.z*sin(xarm_angle);
			new_coord.position.z = between_coord.position.x*sin(xarm_angle) + between_coord.position.z*cos(xarm_angle);
			
			// Calculating the quarternions
			new_coord.orientation.x = sin(roll * 0.5) * cos(pitch * 0.5) * cos(yaw * 0.5) - cos(roll * 0.5) * sin(pitch * 0.5) * sin(yaw * 0.5);
			new_coord.orientation.y = cos(roll * 0.5) * sin(pitch * 0.5) * cos(yaw * 0.5) + sin(roll * 0.5) * cos(pitch * 0.5) * sin(yaw * 0.5);
			new_coord.orientation.z = cos(roll * 0.5) * cos(pitch * 0.5) * sin(yaw * 0.5) - sin(roll * 0.5) * sin(pitch * 0.5) * cos(yaw * 0.5);
			new_coord.orientation.w = cos(roll * 0.5) * cos(pitch * 0.5) * cos(yaw * 0.5) + sin(roll * 0.5) * sin(pitch * 0.5) * sin(yaw * 0.5);
			
			new_coords.poses.push_back(new_coord);
		    counter++;
		}
    }
    pub.publish(new_coords);
    new_coords.poses.clear();
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_translation_node");
    ros::NodeHandle nh;
    
    //Getting parameters from the parameters server
	nh.getParam("/mustikas/camera/offset_x", camera_offset_x);
	nh.getParam("/mustikas/camera/offset_y", camera_offset_y);
	nh.getParam("/mustikas/camera/offset_z", camera_offset_z);
	nh.getParam("/mustikas/camera/angle", cam_angle);
	nh.getParam("/mustikas/xarm/angle", xarm_angle);
	nh.getParam("/mustikas/goal/offset_x", offset_x);
	nh.getParam("/mustikas/goal/offset_y", offset_y);
	nh.getParam("/mustikas/goal/offset_z", offset_z);
	
	roll = 0.0;
	pitch = 3.14159 + xarm_angle;
	yaw = 3.14159;

    pub = nh.advertise<geometry_msgs::PoseArray>("/robot_coords", 10);
    ros::Subscriber sub = nh.subscribe("/object_coords", 10, cb);
    ros::spin();

    return 0;

}




