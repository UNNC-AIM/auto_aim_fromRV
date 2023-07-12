#ifndef pS
#define pS

//#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>
#include <typeinfo>
#include <iostream>

#include "armor_struct.hpp"
using namespace std;
cv::Mat camera_matrix_;
cv::Mat dist_coeffs_;

// Unit: mm
static constexpr float SMALL_ARMOR_WIDTH = 135;
static constexpr float SMALL_ARMOR_HEIGHT = 55;
static constexpr float LARGE_ARMOR_WIDTH = 225;
static constexpr float LARGE_ARMOR_HEIGHT = 55;



// Unit: m
constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;



bool solvePnp(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec, const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
{
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone();
    dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone();

    // Four vertices of armor in 3d
    std::vector<cv::Point3d> small_armor_points_;
    std::vector<cv::Point3d> large_armor_points_;
    
    // Start from bottom left in clockwise order
    // Model coordinate: x forward, y left, z up
    small_armor_points_.emplace_back(cv::Point3d(0, small_half_y, -small_half_z));
    small_armor_points_.emplace_back(cv::Point3d(0, small_half_y, small_half_z));
    small_armor_points_.emplace_back(cv::Point3d(0, -small_half_y, small_half_z));
    small_armor_points_.emplace_back(cv::Point3d(0, -small_half_y, -small_half_z));

    large_armor_points_.emplace_back(cv::Point3d(0, large_half_y, -large_half_z));
    large_armor_points_.emplace_back(cv::Point3d(0, large_half_y, large_half_z));
    large_armor_points_.emplace_back(cv::Point3d(0, -large_half_y, large_half_z));
    large_armor_points_.emplace_back(cv::Point3d(0, -large_half_y, -large_half_z));
    
    std::vector<cv::Point2d> image_armor_points;

    // Fill in image points
    image_armor_points.emplace_back(armor.left_light.bottom);
    image_armor_points.emplace_back(armor.left_light.top);
    image_armor_points.emplace_back(armor.right_light.top);
    image_armor_points.emplace_back(armor.right_light.bottom);

    std::cout<<"armor on image(lb,lt,rt,rb): ("<<armor.left_light.bottom<<","<<armor.left_light.top<<","<<armor.right_light.top<<","<<armor.right_light.bottom<<std::endl;

    //std::cout<<armor.left_light.bottom<<std::endl;

    // Solve pnp
    auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
    return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

//distance between armor center(on the image) and image center(cx,cy) [unit: pixel]
float calculateDistanceToCenter(const cv::Point2f & image_point)
{
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}

#endif