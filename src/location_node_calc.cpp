#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <stdio.h>
#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


ros::Publisher pub;



float depth, x, y, x_corrected, y_corrected;
float fov_vertical = 0.7330; //value in radians. 1 deg = 0.01745 rad
float fov_horizontal = 1.2043; //value in radians

float correction_x = 1.55;


//these are the coordinates of the CV detected objects on the 2D image
int index_x;
int index_y;

bool state = false; //bool used to publish coordinates only when input from CV is received

geometry_msgs::Point object_location;

/*
void cb_picture_coords(const geometry_msgs::PointStamped& msg)
{
    index_x = static_cast<int>(msg.point.x);
    index_y = static_cast<int>(msg.point.y);
    state = true;

}
*/

void callback(const geometry_msgs::PointStamped::ConstPtr& point, const sensor_msgs::ImageConstPtr& image)
{
    index_x = static_cast<int>(point->point.x);
    index_y = static_cast<int>(point->point.y);
    
    state = true;
    
    if (state)
    {
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
        
        // Getting the depth data of the pixel
        auto depth_m = depths_m.at<float>(index_y , index_x);
        
        int half_width = image->width/2;
        int half_height = image->height/2;
        
        // For troubleshooting
        std::cout << "depth: " << depth_m << " half_width: " << half_width << " half_height: " << half_height << " index x: " << index_x << " index y: " << index_y << std::endl;
        std::cout << "\nencoding: " << image->encoding << " is_bigendian: " << image->is_bigendian << " step: " << image->step << std::endl;
        std::cout << "\npoint header: \n" << point->header << "\nimage header: \n" << image->header << std::endl;
        
        
        // Calculating the x coordinate
        // If the point is in the left half of the image, the x value will be positive
        if (index_x <= half_width)
        {
        	x = -depth_m * tan((half_width-index_x) * (fov_horizontal/2) / half_width) ;
        	x_corrected = x - (x*correction_x)*(x*correction_x); 
        } 
        else 
        {
        	x = depth_m * tan((index_x-half_width) * (fov_horizontal/2) / half_width) ;
        	x_corrected = x + (x*correction_x)*(x*correction_x);
        }
        
        // Calculating the y coordinate
        // If the point is in the lower half of the image, the y value will be positive
        if (index_y > half_height)
        {
        	y = depth_m * tan((index_y-half_height) * (fov_vertical/2) / half_height);
        	y_corrected = y + (y*correction_x)*(y*correction_x);
        }
        else
        {
        	y = -depth_m * tan((half_height-index_y) * (fov_vertical/2) / half_height);
        	y_corrected = y - (y*correction_x)*(y*correction_x);
        }
        

        object_location.x = x_corrected ; 
        object_location.y = y_corrected;
        object_location.z = depth_m;

        pub.publish(object_location);
        state = false;
    }
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "mustikas_location_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Point>("/object_coords", 1);
    //ros::Subscriber sub1 = nh.subscribe("/picture_coordinates", 1, cb_picture_coords);
    //ros::Subscriber sub2 = nh.subscribe("/camera/depth/color/points", 1, cb_pointcloud);
    
    message_filters::Subscriber<geometry_msgs::PointStamped> sub1(nh, "/picture_coordinates", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub2(nh, "/camera/depth/image_raw", 1);
    
    // Setting up the synchronization of the topics
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::Image> MySyncPolicy;
    
    //message_filters::TimeSynchronizer<geometry_msgs::PointStamped, sensor_msgs::Image> sync(sub1, sub2, 1000);
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    ros::spin();

    return 0;
}
