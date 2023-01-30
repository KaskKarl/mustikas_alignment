#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <cmath>

//variables to define the camera coordinate origin relative to the robot's origin
float camera_offset_x = 0.0;
float camera_offset_y = -0.2; //positive value means camera is to the left of the arm
float camera_offset_z = 0.0;

float angle = 0.0; //angle in degrees

//additional positioning offsets. Used when collision with the object needs to be avoided
float offset_x = 0.0;
float offset_y = 0.0;
float offset_z = 0.2;


geometry_msgs::Point old_coords, new_coords;
float x, z;

ros::Publisher pub;

void cb(const geometry_msgs::Point msg)
{
    old_coords = msg;

    new_coords.y = (-old_coords.x) + camera_offset_y + offset_y;
    z = old_coords.y - camera_offset_z - offset_z;
    x = old_coords.z + camera_offset_x + offset_x;
	
	//Checking if the object has x coordinate. Objects that are too far or close return an Nan
    if (!std::isnan(old_coords.x)){
    
    	//Using a rotation matrix to rotate the x and z coordinates
    	new_coords.x = x*cos(angle*0.01745) + z*sin(angle*0.01745);
    	new_coords.z = x*sin(angle*0.01745) - z*cos(angle*0.01745);
    	
        pub.publish(new_coords);
    }
    
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_translation_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Point>("/robot_coords", 10);
    ros::Subscriber sub = nh.subscribe("/object_coords", 10, cb);
    ros::spin();

    return 0;

}

