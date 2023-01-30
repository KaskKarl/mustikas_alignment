#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <darknet.h>
#include <DarkHelp.hpp>
#include <stdio.h>
   
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
    geometry_msgs::Point koords;
   
public:
    ImageConverter()
       : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
            &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        koord_pub_ = nh_.advertise<geometry_msgs::Point>("/picture_coordinates", 10);
   
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

        

        //If there is no detected objects, the code will skip getting the coordinates
        if (results.empty()) { goto SHOW_PIC;}     
        
        //Looking through all the detected onjects and outputting only the coordinates of the desired class
        for (auto i : results){
        	//ID of pottedplant is 58
        	if (i.best_class == 58){
        	
        		//Getting the centerpoint coordinates of the ROI
        		koords.x = results[0].rect.x + (results[0].rect.width/2);
        		koords.y = results[0].rect.y + (results[0].rect.height/2);
        		
        		koord_pub_.publish(koords);
        	}
        }
        
        
   
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
