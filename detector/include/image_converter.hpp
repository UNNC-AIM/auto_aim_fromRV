// required headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// the header file that contains the majority of detecting functions
#include "detector_test.hpp"

// name of the window to display the processed images
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  string input_topic, output_topic;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Get topic names from the launch file(or other ROS ways)
    bool ifget1 = nh_.getParam("input_topic",input_topic);
    bool ifget2 = nh_.getParam("output_topic",output_topic);

    // Subscribe to input video feed
    image_sub_ = it_.subscribe(input_topic, 1,
      &ImageConverter::imageCb, this);
    // and publish output video feed
    image_pub_ = it_.advertise(output_topic, 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  // The function to process the input images
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

    int detect_color = 0;
    bool ifget3 = nh_.getParam("detect_color",detect_color);

    // Timing the whole algorithm
    double time1 = static_cast<double>(cv::getTickCount());
    // clock start
    detect(cv_ptr->image, detect_color);
    // clock stop
    double time2 = static_cast<double>(cv::getTickCount());
    double time_use = (time2 - time1)/cv::getTickFrequency()*1000;
    // Output the recorded time to the terminal
    std::cout<<"Time use: "<< time_use <<"ms"<<std::endl;


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};