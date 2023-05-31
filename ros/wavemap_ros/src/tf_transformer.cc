#include "wavemap_ros/tf_transformer.h"

#include <string>

#include <minkindr_conversions/kindr_msg.h>

namespace wavemap {
bool TfTransformer::isTransformAvailable(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time& frame_timestamp) const {
  return tf_buffer_.canTransform(sanitizeFrameId(to_frame_id),
                                 sanitizeFrameId(from_frame_id),
                                 frame_timestamp);
}

bool TfTransformer::waitForTransform(const std::string& to_frame_id,
                                     const std::string& from_frame_id,
                                     const rclcpp::Time& frame_timestamp) {
  return waitForTransformImpl(sanitizeFrameId(to_frame_id),
                              sanitizeFrameId(from_frame_id), frame_timestamp);
}

bool TfTransformer::lookupTransform(const std::string& to_frame_id,
                                    const std::string& from_frame_id,
                                    const rclcpp::Time& frame_timestamp,
                                    Transformation3D& transform) {
  return lookupTransformImpl(sanitizeFrameId(to_frame_id),
                             sanitizeFrameId(from_frame_id), frame_timestamp,
                             transform);
}

std::string TfTransformer::sanitizeFrameId(const std::string& string) {
  if (string[0] == '/') {
    return string.substr(1, string.length());
  } else {
    return string;
  }
}

bool TfTransformer::waitForTransformImpl(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time& frame_timestamp) const {
  // Total time spent waiting for the updated pose
  rclcpp::Duration t_waited = rclcpp::Duration::from_seconds(0.0);
  while (t_waited < transform_lookup_max_time_) {
    if (tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
      return true;
    }
    rclcpp::sleep_for(std::chrono::nanoseconds(transform_lookup_retry_period_.nanoseconds()));

    t_waited = t_waited + transform_lookup_retry_period_;
  }
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
      "Waited %.3fs, but still could not get the TF from %s to %s at timestamp "
      "%u seconds",
      t_waited.seconds(), from_frame_id.c_str(), to_frame_id.c_str(),
      frame_timestamp.seconds());
  return false;
}

bool TfTransformer::lookupTransformImpl(const std::string& to_frame_id,
                                        const std::string& from_frame_id,
                                        const rclcpp::Time& frame_timestamp,
                                        Transformation3D& transform) {
  if (!isTransformAvailable(to_frame_id, from_frame_id, frame_timestamp)) {
    return false;
  }
  geometry_msgs::msg::TransformStamped transform_msg =
      tf_buffer_.lookupTransform(to_frame_id, from_frame_id, frame_timestamp);
  tf::transformMsgToKindrCOPY(transform_msg.transform, &transform);
  return true;
}
}  // namespace wavemap
