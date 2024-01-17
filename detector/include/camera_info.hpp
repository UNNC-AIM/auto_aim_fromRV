#ifndef c_i
#define c_i

#include <vector>
#include <array>
#include <opencv2/core/core.hpp>

std::array<double, 9> tf_robot2camera = {0.0, 0.0, 0.0};

class camera_info
{
private:
    /* data */
public:
    camera_info(/* args */);
    // std::array<double, 9>  camera_matrix = {650.7665088392333, 0.0, 660.144282993784, 0.0, 646.0614689778283, 345.4338458061862, 0.0, 0.0, 1.0};
    // std::vector<double> dist_coeffs = {-0.011321370627002557, 0.021978321621542212, -0.008789247157568734, 0.013912167479008943, 0.0};
    std::array<double, 9>  camera_matrix = {381.5362548828125, 0.0, 314.7439270019531, 0.0, 381.20892333984375, 242.072265625, 0.0, 0.0, 1.0};
    std::vector<double> dist_coeffs = {-0.05815446376800537, 0.0680178552865982, -0.0005185040063224733, -0.0007323402678593993, -0.021494440734386444};

};

camera_info::camera_info(/* args */)
{
}

#endif