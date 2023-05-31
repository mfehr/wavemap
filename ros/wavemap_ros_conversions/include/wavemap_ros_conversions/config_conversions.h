#ifndef WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/config/config_base.h>

namespace wavemap::param::convert {
param::Map toParamMap(rclcpp::Node::SharedPtr nh, const std::string& ns);
param::Array toParamArray(rclcpp::Node::SharedPtr nh, const std::string& ns);

}  // namespace wavemap::param::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_
