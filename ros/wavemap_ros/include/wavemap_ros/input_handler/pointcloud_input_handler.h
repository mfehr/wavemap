#ifndef WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_

#include <memory>
#include <queue>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "wavemap_ros/input_handler/input_handler.h"

namespace wavemap {
class PointcloudInputHandler : public InputHandler {
 public:
  PointcloudInputHandler(const InputHandlerConfig& config,
                         const param::Map& params, std::string world_frame,
                         VolumetricDataStructureBase::Ptr occupancy_map,
                         std::shared_ptr<TfTransformer> transformer,
                         rclcpp::Node::SharedPtr nh);

  InputHandlerType getType() const override {
    return InputHandlerType::kPointcloud;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2& pointcloud_msg) {
    pointcloud_queue_.emplace(pointcloud_msg);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  std::queue<sensor_msgs::msg::PointCloud2> pointcloud_queue_;
  void processQueue() override;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_
