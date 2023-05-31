#ifndef WAVEMAP_ROS_TF_TRANSFORMER_H_
#define WAVEMAP_ROS_TF_TRANSFORMER_H_

#include <map>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <wavemap/common.h>

namespace wavemap {
class TfTransformer {
 public:
  explicit TfTransformer(rclcpp::Node::SharedPtr nh,
                         FloatingPoint tf_buffer_cache_time = 10.f)
      : tf_buffer_(nh->get_clock(), tf2::durationFromSec(tf_buffer_cache_time),
                   nh),
        tf_listener_(tf_buffer_) {}

  // Check whether a transform is available
  bool isTransformAvailable(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const rclcpp::Time& frame_timestamp) const;

  // Waits for a transform to become available, while doing less aggressive
  // polling that ROS's standard tf2_ros::Buffer::canTransform(...)
  bool waitForTransform(const std::string& to_frame_id,
                        const std::string& from_frame_id,
                        const rclcpp::Time& frame_timestamp);

  // Lookup transforms and convert them to Kindr
  bool lookupTransform(const std::string& to_frame_id,
                       const std::string& from_frame_id,
                       const rclcpp::Time& frame_timestamp,
                       Transformation3D& transform);

  // Strip leading slashes if needed to avoid TF errors
  static std::string sanitizeFrameId(const std::string& string);

 private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Transform lookup timers
  // Timeout between each update attempt
  const rclcpp::Duration transform_lookup_retry_period_ =
      rclcpp::Duration::from_seconds(0.02);
  // Maximum time to wait before giving up
  const rclcpp::Duration transform_lookup_max_time_ =
      rclcpp::Duration::from_seconds(0.25);

  bool waitForTransformImpl(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const rclcpp::Time& frame_timestamp) const;
  bool lookupTransformImpl(const std::string& to_frame_id,
                           const std::string& from_frame_id,
                           const rclcpp::Time& frame_timestamp,
                           Transformation3D& transform);
};
}  // namespace wavemap

// NOTE(mfehr): Imported from voliro ROS2 version of mindkindr_conversions

namespace tf2 {

template <typename Scalar>
void fromMsg(const geometry_msgs::msg::Quaternion& q_in,
             Eigen::Quaternion<Scalar>& q_out) {
  Eigen::Quaternion<double> q_out_tmp;
  tf2::fromMsg(q_in, q_out_tmp);
  q_out = q_out_tmp.template cast<Scalar>();
}

template <typename Scalar>
void fromMsg(const geometry_msgs::msg::Vector3& p_in,
             Eigen::Matrix<Scalar, 3, 1>& p_out) {
  Eigen::Vector3d p_out_tmp;
  tf2::fromMsg(p_in, p_out_tmp);
  p_out = p_out_tmp.template cast<Scalar>();
}
}  // namespace tf2
namespace tf {
template <typename Scalar>
void transformMsgToKindrCOPY(
    const geometry_msgs::msg::Transform& msg,
    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  tf2::fromMsg(msg.rotation, rotation);
  tf2::fromMsg(msg.translation, position);

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}
}  // namespace tf

#endif  // WAVEMAP_ROS_TF_TRANSFORMER_H_
