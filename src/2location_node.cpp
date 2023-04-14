#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <mustikas_alignment/PointArray.h>
#include <mustikas_alignment/ROIs.h>
#include <stdio.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


ros::Publisher pub;

sensor_msgs::PointCloud2 input_pointcloud;
sensor_msgs::PointCloud out_pointcloud;

//these are the coordinates of the CV detected objects on the 2D image
int index_x;
int index_y;

int ROI_width;
int ROI_height;

int center_x;
int center_y;

bool state = false; //bool used to publish coordinates only when input from CV is received

mustikas_alignment::PointArray object_locations;
geometry_msgs::Point point;

void callback(const mustikas_alignment::ROIs::ConstPtr& ROI, const sensor_msgs::PointCloud2ConstPtr& PC)
{
	input_pointcloud = *PC;
    sensor_msgs::convertPointCloud2ToPointCloud(input_pointcloud, out_pointcloud);
	
	if (ROI->ROIs.empty()) {goto END_CB;}
	
	for (auto i: ROI->ROIs)
	{
		index_x = i.x_offset;
		index_y = i.y_offset;
		ROI_width = i.width;
		ROI_height = i.height;
		
		center_x = static_cast<int>((float)index_x + ((float)ROI_width/2));
		center_y = static_cast<int>((float)index_y + ((float)ROI_height/2));
		
		point.x = out_pointcloud.points[(1280*center_y)+center_x].x; 
		point.y = out_pointcloud.points[(1280*center_y)+center_x].y;
		point.z = out_pointcloud.points[(1280*center_y)+center_x].z;
		
		object_locations.points.push_back(point);
    
    }
    
    pub.publish(object_locations);
    
    END_CB:
    
    object_locations.points.clear();
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "mustikas_location_node");
    ros::NodeHandle nh;

    pub = nh.advertise<mustikas_alignment::PointArray>("/object_coords", 1);
    
    message_filters::Subscriber<mustikas_alignment::ROIs> sub1(nh, "/picture_coordinates", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "/camera/depth/color/points", 1);
    
    // Setting up the synchronization of the topics
    typedef message_filters::sync_policies::ApproximateTime<mustikas_alignment::ROIs, sensor_msgs::PointCloud2> MySyncPolicy;
    
    //message_filters::TimeSynchronizer<geometry_msgs::PointStamped, sensor_msgs::Image> sync(sub1, sub2, 1000);
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    ros::spin();

    return 0;
}
