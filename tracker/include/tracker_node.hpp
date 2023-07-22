// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
//#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>

// STD
#include <memory>
#include <string>
#include <vector>

#include<auto_aim_msgs/Armor.h>
#include<auto_aim_msgs/Armors.h>
#include<auto_aim_msgs/Target.h>

#include "tracker.hpp"

using tf2_filter = tf2_ros::MessageFilter<auto_aim_msgs::Armors>;
class ArmorTrackerNode
{
  ros::NodeHandle nh;
public:
  explicit ArmorTrackerNode();

  void armorsCallback(const auto_aim_msgs::Armors & armors_msg);

  void publishMarkers(const auto_aim_msgs::Target & target_msg);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  ros::Time last_time;
  double dt_;

  // Armor tracker
  double s2qxyz_, s2qyaw_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;

  // Reset tracker service
  //ros::Service<std_srvs::Trigger>::SharedPtr reset_tracker_srv_;

  std::string target_frame_ = "camera_link";
  // if use TF, change the frame name to the final frame after transform
  // tf2_ros::Buffer tf2_buffer_;

  // Subscriber
  ros::Subscriber armors_sub_;

  // Publisher
  ros::Publisher target_pub_;

  // Visualization marker publisher
  visualization_msgs::Marker position_marker_;
  visualization_msgs::Marker linear_v_marker_;
  visualization_msgs::Marker angular_v_marker_;
  visualization_msgs::Marker armor_marker_;
  ros::Publisher marker_pub_;
};
