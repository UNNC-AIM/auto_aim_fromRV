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
    std::array<double, 9>  camera_matrix = {643.062482, 0.0, 630.882991, 0.0, 642.456525, 363.256627, 0.0, 0.0, 1.0};
    std::vector<double> dist_coeffs = {-0.041794, 0.039360, -0.000261, -0.001079, 0.0};
};

camera_info::camera_info(/* args */)
{
}

#endif