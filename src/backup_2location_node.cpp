#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <stdio.h>
#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <mustikas_alignment/ROIs.h>
#include <mustikas_alignment/PointArray.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


ros::Publisher pub;


float depth, x, y, x_corrected, y_corrected;
float fov_vertical = 0.7330; //value in radians. 1 deg = 0.01745 rad
float fov_horizontal = 1.2043; //value in radians

float correction_x = 1.55;


//these are the coordinates of the upper left corner of the ROI
int index_x;
int index_y;

//these are the dimensions of the ROI
int ROI_width;
int ROI_height;

int center_x;
int center_y;

mustikas_alignment::PointArray object_locations;

/*
void cb_picture_coords(const geometry_msgs::PointStamped& msg)
{
    index_x = static_cast<int>(msg.point.x);
    index_y = static_cast<int>(msg.point.y);
    state = true;

}
*/

void callback(const mustikas_alignment::ROIs::ConstPtr& ROI, const sensor_msgs::ImageConstPtr& image)
{
	int counter = 0;
	object_locations.points.resize(10);
	
	// Converting the message to openCV format        
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	    	cv_ptr = cv_bridge::toCvCopy(image);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	    	ROS_ERROR("cv_bridge exception: %s", e.what());
	    	return;
	    }
	    
	    // Creating a matrix of the image. Depth units in mm
	    cv::Mat depths_mm = cv_ptr->image;
	    
	    // Creating a matrix to hold the depth data in m
	    cv::Mat depths_m;
	    
	    // Converting the depths_mm matrix values to depths_m matrix. Values are now in m
	    depths_mm.convertTo(depths_m, CV_32FC1, 0.001);
	
	
	//loop through all the coordinates that are given
	for (auto i : ROI->ROIs){
		index_x = i.x_offset;
		index_y = i.x_offset;
		ROI_width = i.width;
		ROI_height = i.height;
		
		center_x = index_x + (ROI_width/2);
		center_y = index_y + (ROI_height/2);
		
		float ROI_depths[ROI_height+ROI_width];
		float sum_depth;
	    
	    /*
	    // Getting the depth data of the area
	    for (int j=0; j<=width; j++)
	    {
	    	for (int k=0; k<=height; k++)
	    	{
	    		ROI_depths[j+k] = depths_m.at<float>(index_y+k , index_x+j);
	    		sum_depth += depths_m.at<float>(index_y+k , index_x+j);
	    	}
	    }
	    
	    //Finding the average depth
	    auto depth_m = sum_depth/sizeof(ROI_depths);
	    */
	    
	    auto depth_m = depths_m.at<float>(center_y , center_x);
	    
	    int half_width = image->width/2;
	    int half_height = image->height/2;
	    
	    
	    // Calculating the x coordinate
	    // If the point is in the left half of the image, the x value will be positive
	    if (center_x<= half_width)
	    {
	    	x = -depth_m * tan((half_width-center_x) * (fov_horizontal/2) / half_width) ;
	    	x_corrected = x - (x*correction_x)*(x*correction_x); 
	    } 
	    else 
	    {
	    	x = depth_m * tan((center_x-half_width) * (fov_horizontal/2) / half_width) ;
	    	x_corrected = x + (x*correction_x)*(x*correction_x);
	    }
	    
	    // Calculating the y coordinate
	    // If the point is in the lower half of the image, the y value will be positive
	    if (center_y > half_height)
	    {
	    	y = depth_m * tan((center_y-half_height) * (fov_vertical/2) / half_height);
	    	y_corrected = y + (y*correction_x)*(y*correction_x);
	    }
	    else
	    {
	    	y = -depth_m * tan((half_height-center_y) * (fov_vertical/2) / half_height);
	    	y_corrected = y - (y*correction_x)*(y*correction_x);
	    }
	    

	    object_locations.points[counter].x = x_corrected ; 
	    object_locations.points[counter].y = y_corrected;
	    object_locations.points[counter].z = depth_m;
	    
	    std::cout << "mustikas: " << counter << " x: " << x_corrected << " depth: " << depth_m << std::endl;

	    counter++;
      
    }
    
    pub.publish(object_locations);
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "mustikas_location_node");
    ros::NodeHandle nh;

    pub = nh.advertise<mustikas_alignment::PointArray>("/object_coords", 1);
    //ros::Subscriber sub1 = nh.subscribe("/picture_coordinates", 1, cb_picture_coords);
    //ros::Subscriber sub2 = nh.subscribe("/camera/depth/color/points", 1, cb_pointcloud);
    
    message_filters::Subscriber<mustikas_alignment::ROIs> sub1(nh, "/picture_coordinates", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub2(nh, "/camera/depth/image_raw", 1);
    
    // Setting up the synchronization of the topics
    typedef message_filters::sync_policies::ApproximateTime<mustikas_alignment::ROIs, sensor_msgs::Image> MySyncPolicy;
    
    //message_filters::TimeSynchronizer<geometry_msgs::PointStamped, sensor_msgs::Image> sync(sub1, sub2, 1000);
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    ros::spin();

    return 0;
}
