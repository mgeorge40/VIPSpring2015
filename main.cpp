//THIS IS THE DEPTH PROCESSING CODE

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
 
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr_depth_;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects grayscale images to use MONO8 channel order.
	cv_ptr_depth_ = cv_bridge::toCvCopy(original_image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

// This code converts and scales the image to grayscale.
float min_range_ = 0;
float max_range_ = 10;
  cv::Mat img(cv_ptr_depth_->image.rows, cv_ptr_depth_->image.cols, CV_8UC1);
    for(int i = 0; i < cv_ptr_depth_->image.rows; i++)
    {
        float* Di = cv_ptr_depth_->image.ptr<float>(i);
        char* Ii = img.ptr<char>(i);
        for(int j = 0; j < cv_ptr_depth_->image.cols; j++)
        {   
            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
        }   
    }

int val = img.at<cv::Vec3b>(100,100)[0];
int tol = 2;

    //Go through all the rows
    for(int i= 0; i<cv_ptr_depth_->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr_depth_->image.cols; j++)
        {
            {
		// If the pixel you're looking at is the same color as the location you want to track, turn it white so we can see it.
		if ( (img.at<cv::Vec3b>(i,j)[0] > (val - tol)) && (img.at<cv::Vec3b>(i,j)[0] < (val + tol)) )
		{
			img.at<cv::Vec3b>(i,j)[0] = 255;
		}
		
            }
        }
    }

    //Display the image using OpenCV
    cv::imshow(WINDOW, img);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        pub.publish(cv_ptr_depth_->toImageMsg());
}
 
int main(int argc, char **argv)
{
        ros::init(argc, argv, "image_processor");
 ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
        image_transport::Subscriber sub = it.subscribe("/camera/depth/image", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
        pub = it.advertise("camera/image_processed", 1);
        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
