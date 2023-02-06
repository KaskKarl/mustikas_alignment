#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <stdio.h>


ros::Publisher pub;

sensor_msgs::PointCloud2 input_pointcloud;
sensor_msgs::PointCloud out_pointcloud;

//these are the coordinates of the CV detected objects on the 2D image
int index_x;
int index_y;

bool state = false; //bool used to publish coordinates only when input from CV is received

geometry_msgs::Point object_location;

void cb_picture_coords(const geometry_msgs::PointStamped& msg)
{
    index_x = static_cast<int>(msg.point.x);
    index_y = static_cast<int>(msg.point.y);
    state = true;

}

void cb_pointcloud(const sensor_msgs::PointCloud2& msg)
{
    if (state)
    {
        input_pointcloud = msg;
        sensor_msgs::convertPointCloud2ToPointCloud(input_pointcloud, out_pointcloud);

        object_location.x = out_pointcloud.points[(1280*index_y)+index_x].x; 
        object_location.y = out_pointcloud.points[(1280*index_y)+index_x].y;
        object_location.z = out_pointcloud.points[(1280*index_y)+index_x].z;

        pub.publish(object_location);
        state = false;
    }
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "mustikas_location_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Point>("/object_coords", 1);
    ros::Subscriber sub1 = nh.subscribe("/picture_coordinates", 1, cb_picture_coords);
    ros::Subscriber sub2 = nh.subscribe("/camera/depth/color/points", 1, cb_pointcloud);
    ros::spin();

    return 0;
}
