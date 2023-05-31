#include "wavemap_ros_conversions/config_conversions.h"

namespace wavemap::param::convert {
param::Map toParamMap(rclcpp::Node::SharedPtr nh, const std::string& ns) {
  rcl_interfaces::msg::ListParametersResult list_params_result =
      nh.list_parameters({ns}, 1);

  param::Map param_map;
  for (int i = 0; i < list_params_result.names.size(); ++i) {
    rclcpp::Parameter ros_param = nh->get_parameter(list_params_result.prefixes[i] + "/" + list_params_result.names[i], xml_rpc_value);
    param_map.emplace(list_params_result.names[i], toParamValue(ros_param));
  }
  return param_map;
}

param::Array toParamArray(rclcpp::Node::SharedPtr nh, const std::string& ns) {
  rcl_interfaces::msg::ListParametersResult list_params_result =
      nh.list_parameters({ns}, 1);

  param::Array array;
  for (int i = 0; i < list_params_result.names.size(); ++i) {
    rclcpp::Parameter ros_param = nh.get_parameter(list_params_result.prefixes[i] + "/" + list_params_result.names[i], xml_rpc_value);
    array.emplace_back(toParamValue(ros_param));
  }
  return param_map;

  param::Array array;
  array.reserve(xml_rpc_value.size());
  for (int idx = 0; idx < xml_rpc_value.size(); ++idx) {  // NOLINT
    array.template emplace_back(toParamValue(xml_rpc_value[idx]));
  }
  return array;
}

param::Value toParamValue(  // NOLINT
    const rclcpp::Parameter& ros_param) {
  switch (ros_param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return param::Value(ros_param.as_bool());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return param::Value(ros_param.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param::Value(ros_param.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return param::Value(ros_param.as_string());
    case rclcpp::ParameterType::TypeArray:
      return param::Value(toParamArray(xml_rpc_value));
    case rclcpp::ParameterType::TypeStruct:
      return param::Value(toParamMap(xml_rpc_value));
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Encountered invalid type while parsing ROS params.");
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Encountered unknown type while parsing ROS params.");
      break;
  }

  // On error, return an empty array
  return param::Value(param::Array{});
}
}  // namespace wavemap::param::convert
