#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace arcs_cohort_sensor_preprocessor {

class PointCloudTransformer : public rclcpp::Node {
public:
  explicit PointCloudTransformer(const rclcpp::NodeOptions & options);

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  std::string target_frame_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace arcs_cohort_sensor_preprocessor
