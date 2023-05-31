#ifndef WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_FACTORY_H_
#define WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap_ros/input_handler/input_handler.h"

namespace wavemap {
class InputHandlerFactory {
 public:
  static std::unique_ptr<InputHandler> create(
      const param::Map& params, std::string world_frame,
      VolumetricDataStructureBase::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer, rclcpp::Node::SharedPtr nh,
      std::optional<InputHandlerType> default_input_handler_type =
          std::nullopt);

  static std::unique_ptr<InputHandler> create(
      InputHandlerType input_handler_type, const param::Map& params,
      std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer, rclcpp::Node::SharedPtr nh);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_FACTORY_H_
