// required headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//self-defined messages
#include<auto_aim_msgs/Armor.h>
#include<auto_aim_msgs/Armors.h>

// the header file that contains the majority of detecting functions
#include "detector_test.hpp"
#include "armor_struct.hpp"

// name of the window to display the processed images
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  string input_topic, output_topic;
  ros::Publisher armors_pub = nh_.advertise<auto_aim_msgs::Armors>("/detector/armors", 100);

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
    vector<Armor> armors = detect(cv_ptr->image, detect_color);
    // clock stop
    double time2 = static_cast<double>(cv::getTickCount());
    double time_use = (time2 - time1)/cv::getTickFrequency()*1000;
    // Output the recorded time to the terminal
    std::cout<<"Time use: "<< time_use <<"ms"<<std::endl;

    // Pack the infomation of armors into ros message
    auto_aim_msgs::Armors armors_msg; //message contains all the armors
    auto_aim_msgs::Armor armor_msg;   //temporary message, store the information of each armor
    for (const auto & armor : armors){

      armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
      armor_msg.number = armor.number;

      // Try tramsform manually(for tracker debug), but seems useless :(
      armor_msg.pose.position.x = armor.position.at<double>(2);
      armor_msg.pose.position.y = -armor.position.at<double>(0);
      armor_msg.pose.position.z = -armor.position.at<double>(1);

      // tf2::Quaternion rotation, q_new;
      // rotation.setRPY(0.436, -1.570, 1.134);
      // q_new = rotation*armor.rotation_q;

      // armor_msg.pose.orientation.x = q_new.getX();
      // armor_msg.pose.orientation.y = q_new.getY();
      // armor_msg.pose.orientation.z = q_new.getZ();
      // armor_msg.pose.orientation.w = q_new.getW();

      armor_msg.pose.orientation.x = armor.rotation_q.getX();
      armor_msg.pose.orientation.y = armor.rotation_q.getY();
      armor_msg.pose.orientation.z = armor.rotation_q.getZ();
      armor_msg.pose.orientation.w = armor.rotation_q.getW();

      armor_msg.distance_to_image_center = armor.distance_to_image_center;

      // Add the armor to armors_msg
      armors_msg.armors.emplace_back(armor_msg);
    }
    
    armors_pub.publish(armors_msg);

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
  }
};