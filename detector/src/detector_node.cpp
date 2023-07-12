#include<ros/ros.h> 
#include<iostream> 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
 
//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

//include relevant hpp files
#include "image_converter.hpp"

using namespace cv;
using namespace std;

// run the ros node, call the image topic subscriber and publisher
int main(int argc, char** argv)
{
    ros::init(argc, argv, "detector");
    
    // All in its default constructor
    ImageConverter ic;

    ros::spin();
}
