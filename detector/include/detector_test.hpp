#ifndef d_t // to prevent repeatedly including this file
#define d_t

#include<iostream>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<ros/package.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2/convert.h>

#include"armor_struct.hpp"
#include"classifier.hpp"
#include"pnpSolver.hpp"
#include"camera_info.hpp"
#include"transform.hpp"

using namespace cv;
using namespace std;

#define binary_thres 200 //binary threshold
#define RED 0
#define BLUE 1

//parameters of the lights
struct LightParams
  {
    // width / height (to filter the lights)
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
  };
// parameters copied from Chenjun ：)
LightParams l = {0.1, 0.4, 40.0};

struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance (check if the light pairs are legal with the detected number on it)
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };
// parameters copied from Chenjun, too ：)
ArmorParams a = {0.7, 0.8, 3.2, 3.2, 7, 35.0};

// Declare functions (all be implemented below)
Mat preprocessImage(const Mat & rgb_img);
vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
bool isLight(const Light & light);
void drawResults(cv::Mat & img, vector<Light> & lights, vector<Armor> & armors);
ArmorType isArmor(const Light & light_1, const Light & light_2);
bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
std::vector<Armor> matchLights(const std::vector<Light> & lights, int detect_color);


// Compressed detecting function (CONTAINS ALL DETECTOR PROCESS)*
void detect(Mat & image, const int detect_color){
    if(image.empty())
    {
        cout<<"open image error!"<<endl;
    }
    //imshow("original", image);

    // BGR image -> Binary image, according to the binary_thres defined above
    Mat binaryImg = preprocessImage(image);
    //imshow("binary", binaryImg);

    // Find the possible lights in the image
    vector<Light> lights = findLights(image, binaryImg);
    
    // Match the lights, if legal -> add this armor to armors
    vector<Armor> armors = matchLights(lights, detect_color);

    //------------initialize parameters of classifier-------------
    auto pkg_path = ros::package::getPath("detector");
    auto model_path = pkg_path + "/model/mlp.onnx";
    auto label_path = pkg_path + "/model/label.txt";
    double threshold = 0.7; // the threshold of the confidence of the number got by classification
    // If the result of classification is "negative", ignore this armor
    std::vector<std::string> ignore_classes =std::vector<std::string>{"negative"};

    // Build a classifier
    std::unique_ptr<NumberClassifier> classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);
    // Do classification
    if(!armors.empty()){
      // Extract the standard image of the armor-number from the image, stored in each armor
      classifier->extractNumbers(image, armors);
      // for (auto & armor : armors){
      //   imshow("numImg", armor.number_img);
      // }

      // classify the number of the given armor-number image, using the model in the given path
      classifier->classify(armors);
    }
    
    // Calculate transform (from camera to armor) and
    // distance (from armor center in the image to image center)
    for (auto & armor : armors) {
      // initialize the vector to store the calculated rotation and translation
      cv::Mat rvec, tvec;
      // to get the camera_information stored in camera_info class
      camera_info ci;

      // Calculate the transform use cv::solvePnP (wrapped in the function solvePnp below)
      bool success = solvePnp(armor, rvec, tvec, ci.camera_matrix, ci.dist_coeffs);
      if (success) {

        // Fill pose
        armor.position = tvec;

        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);

        // Fill rotation quaternion
        armor.rotation_q = tf2_q;

        // Fill the distance to image center
        armor.distance_to_image_center = calculateDistanceToCenter(armor.center);
        
      }
    }

    // debug: output the number of armors
    cout<<"number of detected armors: "<<armors.size()<<endl;

    // --------- select the closest armor (distance to the image center) --------
    if(!armors.empty()){
      Armor tracked_armor;
      float min_distance = armors[0].distance_to_image_center;
      tracked_armor = armors[0];
      // repeatly compare armor[0]
      for (const auto & armor : armors) {
          if (armor.distance_to_image_center < min_distance) {
              min_distance = armor.distance_to_image_center;
              tracked_armor = armor;
          }
      }

      // Transform the position of the armor from camera coordinate system
      // to the required coordinate system
      // ******* suppose no rotation from camera to robot *******
      tracked_armor.position.at<double>(0) = tracked_armor.position.at<double>(0) + tf_robot2camera[0];
      tracked_armor.position.at<double>(1) = tracked_armor.position.at<double>(1) + tf_robot2camera[1];
      tracked_armor.position.at<double>(2) = tracked_armor.position.at<double>(2) + tf_robot2camera[2];

      // Output the target position and the distance from armor center to image center
      cout<<"position(x,y,z) = ("<<tracked_armor.position.at<double>(0)<<","<<tracked_armor.position.at<double>(1)<<","<<tracked_armor.position.at<double>(2)<<")"<<endl;
      cout<<"distance to image center = "<<tracked_armor.distance_to_image_center<<endl;
    }

    // Draw the lights, armors as well as the detected number on the armor on the image
    drawResults(image, lights, armors);
    imshow("drawn", image);

}

//preprocess image: rgb -> binary
Mat preprocessImage(const Mat & rgb_img){
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

    return binary_img;
}

//find lights
vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  //this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;   //跳过太小的轮廓（即该轮廓上的点数小于5）

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour (BGR mode)
              sum_r += roi.at<cv::Vec3b>(i, j)[2];
              sum_b += roi.at<cv::Vec3b>(i, j)[0];
            }
          }
        }
        //cout<<"sum_r: "<<sum_r<<"; sum_b: "<<sum_b<<endl;
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  return is_light;
}

//draw result detected lights and armors on the picture
void drawResults(cv::Mat & img, vector<Light> & lights, vector<Armor> & armors)
{
  // Draw Lights
  for (const auto & light : lights) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }

  // Draw the center of the image
  cv::circle(img, cv::Point(630.882991, 363.256627), 10, cv::Scalar(255, 0, 255), 5);
}

//match lights
std::vector<Armor> matchLights(const std::vector<Light> & lights, int detect_color)
{
  std::vector<Armor> armors;

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) {
        //cout<<"different color, skip"<<endl;
        continue;
      }
      if (containLight(*light_1, *light_2, lights)) {
        //cout<<"contained, skip"<<endl;
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      //cout<<(type == ArmorType::INVALID)<<endl;
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights(若这两个灯带围成的区域内还有灯带，就跳过)
bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  return type;
}

#endif