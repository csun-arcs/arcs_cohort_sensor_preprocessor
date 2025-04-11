#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip> // for std::setprecision
#include <sstream> // for std::ostringstream

#include "arcs_cohort_sensor_preprocessor/pointcloud_transformer.hpp"

namespace arcs_cohort_sensor_preprocessor {

PointCloudTransformer::PointCloudTransformer(const rclcpp::NodeOptions &options)
    : Node("pointcloud_transformer", options) {
  this->declare_parameter<std::string>("input_topic");
  this->declare_parameter<std::string>("output_topic");
  this->declare_parameter<std::string>("target_frame");

  std::string input_topic = this->get_parameter("input_topic").as_string();
  std::string output_topic = this->get_parameter("output_topic").as_string();
  target_frame_ = this->get_parameter("target_frame").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&PointCloudTransformer::cloud_callback, this,
                std::placeholders::_1));

  pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

  RCLCPP_INFO(this->get_logger(),
              "PointCloudTransformer started. Transforming '%s' to frame '%s'",
              input_topic.c_str(), target_frame_.c_str());
}

void PointCloudTransformer::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  if (cloud_msg->header.frame_id == target_frame_) {
    pub_->publish(*cloud_msg); // no transform needed
    return;
  }

  try {
    // Lookup transform
    geometry_msgs::msg::TransformStamped transformStamped =
        tf_buffer_->lookupTransform(target_frame_, cloud_msg->header.frame_id,
                                    cloud_msg->header.stamp);

    RCLCPP_DEBUG(this->get_logger(),
                 "Transforming cloud from [%s] to [%s] at time %.3f",
                 cloud_msg->header.frame_id.c_str(), target_frame_.c_str(),
                 rclcpp::Time(cloud_msg->header.stamp).seconds());

    Eigen::Affine3d transform = tf2::transformToEigen(transformStamped);
    Eigen::Matrix4f transform_matrix = transform.cast<float>().matrix();

    // Pretty print the matrix
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "\nTransform matrix:\n"
        << transform_matrix;
    RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());

    sensor_msgs::msg::PointCloud2 transformed_msg;

    // Transform point cloud
    pcl_ros::transformPointCloud(transform_matrix, *cloud_msg, transformed_msg);

    // Update header
    transformed_msg.header.stamp = cloud_msg->header.stamp;
    transformed_msg.header.frame_id = target_frame_;

    // Publish transformed point cloud
    pub_->publish(transformed_msg);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
  }
}

} // namespace arcs_cohort_sensor_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
    arcs_cohort_sensor_preprocessor::PointCloudTransformer)
