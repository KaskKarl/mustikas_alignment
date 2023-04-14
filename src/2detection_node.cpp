#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <darknet.h>
#include <DarkHelp.hpp>
#include <stdio.h>
#include <string.h>

#include <mustikas_alignment/ROIs.h>
   
static const std::string OPENCV_WINDOW = "Image window";


//Loading the Darknet network with specified parameters
DarkHelp::NN darkhelp("/home/peko/darknet/cfg/yolov4-tiny.cfg", "/home/peko/darknet/yolov4-tiny.weights", "/home/peko/darknet/data/coco.names");
   
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher koord_pub_;

    //Varible to hold the found coordinates
    mustikas_alignment::ROIs detections;
    sensor_msgs::RegionOfInterest one;
    
    
   
public:
    ImageConverter()
       : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
            &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        koord_pub_ = nh_.advertise<mustikas_alignment::ROIs>("/picture_coordinates", 10);
   
        cv::namedWindow(OPENCV_WINDOW);
    }
   
    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
   
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
   
        // Running darknet detection and annotating the image
        auto results = darkhelp.predict(cv_ptr->image);
        cv::Mat image1 = darkhelp.annotate();
        
        

        int counter = 0;
        //detections.ROIs.resize(10);

        //If there is no detected objects, the code will skip getting the coordinates
        if (results.empty()) { goto SHOW_PIC;}  
           
        detections.header = msg->header;
        
        //Looking through all the detected objects and outputting only the coordinates of the desired class
        for (auto i : results){
        	//ID of pottedplant is 58
        	//ID of bottle is 39
        	if (i.best_class == 58){
        	/*
        		//Getting the centerpoint coordinates of the ROI
        		detections.ROIs[counter].x_offset = i.rect.x;
        		detections.ROIs[counter].y_offset = i.rect.y;
        		detections.ROIs[counter].height = i.rect.height;
        		detections.ROIs[counter].width = i.rect.width;
        		detections.header = msg->header;
        		*/
        		one.x_offset = i.rect.x;
        		one.y_offset = i.rect.y;
        		one.height = i.rect.height;
        		one.width = i.rect.width;
        		
        		detections.ROIs.push_back(one);
        		counter++;
        	}
        	
        }
        
        
        
   		koord_pub_.publish(detections);
   		
   		//clearing object locations
		detections.ROIs.clear();
		
        SHOW_PIC:
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, image1);
        cv::waitKey(3);
   
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
        
    }
};
   
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "mustikas_detection_node");
        ImageConverter ic;
        ros::spin();
        return 0;
    }
