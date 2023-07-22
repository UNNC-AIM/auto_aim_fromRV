// #include<ros/ros.h>
// #include<auto_aim_msgs/Armor.h>
// #include<auto_aim_msgs/Armors.h>
// #include<auto_aim_msgs/Target.h>

// #include "tracker.hpp"
// double dt_;
// ros::Time last_time;

// void armorsCallback(const auto_aim_msgs::Armors armors_msg){  

//     // to calculate dt_: dt_ = time - last_time
//     ros::Time time = armors_msg.header.stamp;

//     auto_aim_msgs::Target target_msg;

//     // f - Process function
//     auto f = [](const Eigen::VectorXd & x) {
//         Eigen::VectorXd x_new = x;
//         x_new(0) += x(1) * dt_;
//         x_new(2) += x(3) * dt_;
//         x_new(4) += x(5) * dt_;
//         x_new(6) += x(7) * dt_;
//         return x_new;
//     };
//     // J_f - Jacobian of process function
//     auto j_f = [](const Eigen::VectorXd &) {
//         Eigen::MatrixXd f(9, 9);
//         // clang-format off
//         f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
//             0,   1,   0,   0,   0,   0,   0,   0,   0,
//             0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
//             0,   0,   0,   1,   0,   0,   0,   0,   0,
//             0,   0,   0,   0,   1,   dt_, 0,   0,   0,
//             0,   0,   0,   0,   0,   1,   0,   0,   0,
//             0,   0,   0,   0,   0,   0,   1,   dt_, 0,
//             0,   0,   0,   0,   0,   0,   0,   1,   0,
//             0,   0,   0,   0,   0,   0,   0,   0,   1;
//         // clang-format on
//         return f;
//     };
//     // h - Observation function
//     auto h = [](const Eigen::VectorXd & x) {
//         Eigen::VectorXd z(4);
//         double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
//         z(0) = xc - r * cos(yaw);  // xa
//         z(1) = yc - r * sin(yaw);  // ya
//         z(2) = x(4);               // za
//         z(3) = x(6);               // yaw
//         return z;
//     };
//     // J_h - Jacobian of observation function
//     auto j_h = [](const Eigen::VectorXd & x) {
//         Eigen::MatrixXd h(4, 9);
//         double yaw = x(6), r = x(8);
//         // clang-format off
//         //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
//         h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
//             0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
//             0,   0,   0,   0,   1,   0,   0,          0,   0,
//             0,   0,   0,   0,   0,   0,   1,          0,   0;
//         // clang-format on
//         return h;
//     };

//     // Update tracker
//     if (tracker->tracker_state == Tracker::LOST) {
//         tracker->init(armors_msg);
//         target_msg.tracking = false;
//     } else {
//         dt_ = (time - last_time).toSec();
//         tracker->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
//         tracker->update(armors_msg);
//     }
// }

// int main(int argc, char **argv){
//     ros::init(argc, argv, "tracker");
//     ros::NodeHandle n;
//     /* Get the TF (from camera to robot) from ros parameters
//      *only translation
//     float translation[3] = {0.0, 0.0, 0.0};
//     bool ifget = n.getParam("translation",translation);*/
//     last_time = ros::Time::now();    

//     // Initialize tracker, default: max_match_distance = 0.15, max_match_yaw_diff = 1.0
//     std::unique_ptr<Tracker> tracker = std::make_unique<Tracker>(0.15, 1.0);  
//     // set tracking_thres to 5 (default value)
//     tracker->tracking_thres = 5;

//     // Subscribe the information of armors published by detector,
//     // use callback function 'armorsCallback' to process the messages
//     ros::Subscriber sub = n.subscribe("/armors", 100, armorsCallback);
//     ros::spin();
//     return 0;
// }