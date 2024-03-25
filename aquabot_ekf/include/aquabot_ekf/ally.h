#ifndef ALLY_H
#define ALLY_H

#include <aquabot_ekf/gps2enu.h>
#include <aquabot_ekf/covariances.h>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace aquabot_ekf
{

struct Ally
{
  inline static int id{0};
  rclcpp::Node* node;
  toENU* enu;

  // forwarded messages
  PoseWithCovarianceStamped pose;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub;

  Ally(rclcpp::Node* node, toENU* enu) : node{node}, enu{enu}
  {
    // init pose and imu messages
    pose.header.frame_id = "world";

    for(auto i: {0, 7})
      pose.pose.covariance[i] = Covariances::xy;
    pose.pose.covariance[14] = Covariances::z;
    for(auto i: {21, 28, 35})
      pose.pose.covariance[i] = Covariances::rpy;

    const auto frame = "friend" + std::to_string(id++);
    pose_pub = node->create_publisher<PoseWithCovarianceStamped>(frame + "/gps_pose", rclcpp::SensorDataQoS());
  }

  void updatePose(const Pose &gps)
  {
    if(!enu->isInit())
      return;

    enu->transform(gps.position.x, gps.position.y, gps.position.z,
                   pose.pose.pose.position);        

    pose.pose.pose.orientation = gps.orientation;
    pose.header.stamp = node->get_clock()->now();
    pose_pub->publish(pose);
  }
};


}


#endif // ALLY_H
