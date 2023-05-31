#ifndef WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_

#include <rclcpp/rclcpp.hpp>

namespace wavemap::convert {
inline rclcpp::Time nanoSecondsToRosTime(uint64_t nsec) {
  uint64_t kSecToNsec = 1000000000ull;
  return rclcpp::Time(nsec);
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_
