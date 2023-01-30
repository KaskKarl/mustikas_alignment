#include <ros/ros.h>
#include <geometry_msgs/Point.h>

//variables to define the camera coordinate origin relative to the robot's origin
float camera_offset_y = -0.2;

//additional positioning offsets. Used when collision with the object needs to be avoided
float offset_z = 0.2;

geometry_msgs::Point old_coords, new_coords;


ros::Publisher pub;

void cb(const geometry_msgs::Point msg)
{
    old_coords = msg;

    new_coords.y = -(old_coords.x + 0.2);
    new_coords.z = (-old_coords.y + 0.2);
    new_coords.x = old_coords.z;

    if (!std::isnan(old_coords.x)){
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

