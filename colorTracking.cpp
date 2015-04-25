//THIS IS THE COLOR TRACKING CODE

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
    //cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_depth_;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr_depth_ = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

// Set the RGB values for the specific color you're tracking. In this case, the color is pink
int val1 = 44;
int val2 = 114;
int val3 = 74;
int tol = 17; 
for(int k=0; k<cv_ptr_depth_->image.channels(); k++)

    //Go through all the rows
    for(int i= 0; i<cv_ptr_depth_->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr_depth_->image.cols; j++)
        {
            {
		// If the first layer color at that pixel is within the tolerated range, continue and examine the second color.
		if ((cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 1] > (val1 - tol)) && (cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 1] < (val1 + tol)))
		{
			// If the second color pixel is also the correct color, continue and check the third.
			if ((cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 2] > (val2 - tol)) && (cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 2] < (val2 + tol))) 
			{
				// If all the colors are within the tolerated range, this is the color you want to track. Change it to red on the image.
				if ((cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 3]> (val3 - tol)) && (cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 3] < (val3 + tol)))
				{
				cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 1] = 0;
				cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 2] = 255;
				cv_ptr_depth_->image.data[i*cv_ptr_depth_->image.rows*4+j*3 + 3] = 0;
				}
			}
		}
		
            }
        }
    }

    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr_depth_->image);
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
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
        pub = it.advertise("camera/image_processed", 1);
ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
