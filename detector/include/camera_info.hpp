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
    std::array<double, 9>  camera_matrix = {650.7665088392333, 0.0, 660.144282993784, 0.0, 646.0614689778283, 345.4338458061862, 0.0, 0.0, 1.0};
    std::vector<double> dist_coeffs = {-0.011321370627002557, 0.021978321621542212, -0.008789247157568734, 0.013912167479008943, 0.0};
    // std::array<double, 9>  camera_matrix = {6382.98974609375, 0.0, 314.7439270019531, 0.0, 382.66119384765625, 242.072265625, 0.0, 0.0, 1.0};
    // std::vector<double> dist_coeffs = {-0.041794, 0.039360, -0.000261, -0.001079, 0.0};
};

camera_info::camera_info(/* args */)
{
}

#endif