#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
 
//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

//include relevant hpp files
#include "detector_test.hpp"
#include "image_converter.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // 测试图片则对*线框起来的部分取消注释
    //**********************************************************************************
    // ros::init(argc, argv, "opencv_test");
    // if(argc != 2)
    // {
    //     cout<<"usage:rosrun opencv_test opencv_test_node <path of picture>"<<endl;
    // }  //图片路径最好为绝对路径

    // double time1 = static_cast<double>(cv::getTickCount());
    // Mat image = cv::imread(argv[1]);
    
    // //开始运行任务
    // detect(image);
    // //结束运行任务
    // double time2 = static_cast<double>(cv::getTickCount());
    
    // double time_use = (time2 - time1)/cv::getTickFrequency()*1000;
    // std::cout<<"Time use: "<< time_use <<"ms"<<std::endl;//输出运行时间
    //**********************************************************************************

    // 要测试视频就对-线框起来的部分取消注释
    //----------------------------------------------------------------------------------
    // VideoCapture capture("rs_v.webm");
 
	// while (true)
	// {
	// 	Mat frame;
	// 	capture >> frame;
    //     detect(frame);
	// 	//imshow("读取视频", frame);
	// 	waitKey(3);	//延时30
	// }
    //----------------------------------------------------------------------------------
   

    ros::spin();
}

