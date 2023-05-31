#ifndef WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_

#include <memory>
#include <string>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/image.h>
#include <wavemap/data_structure/pointcloud.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/integrator/integrator_base.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <wavemap_msgs/msg/map.hpp>

#include "wavemap_ros/tf_transformer.h"
#include "wavemap_ros/utils/timer.h"

namespace wavemap {
struct InputHandlerType : public TypeSelector<InputHandlerType> {
  using TypeSelector<InputHandlerType>::TypeSelector;

  enum Id : TypeId { kPointcloud, kDepthImage, kLivox };

  static constexpr std::array names = {"pointcloud", "depth_image", "livox"};
};

struct InputHandlerConfig : public ConfigBase<InputHandlerConfig, 10> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  FloatingPoint processing_retry_period = 0.05f;
  FloatingPoint max_wait_for_pose = 1.f;

  std::string sensor_frame_id;  // Leave blank to use frame_id from msg header
  std::string image_transport_hints = "raw";
  FloatingPoint depth_scale_factor = 1.f;
  FloatingPoint time_delay = 0.f;

  std::string reprojected_pointcloud_topic_name;  // Leave blank to disable
  std::string projected_range_image_topic_name;   // Leave blank to disable

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class InputHandler {
 public:
  InputHandler(const InputHandlerConfig& config, const param::Map& params,
               std::string world_frame,
               VolumetricDataStructureBase::Ptr occupancy_map,
               std::shared_ptr<TfTransformer> transformer,
               rclcpp::Node::SharedPtr nh);
  virtual ~InputHandler() = default;

  virtual InputHandlerType getType() const = 0;
  const InputHandlerConfig& getConfig() const { return config_; }

  bool shouldPublishReprojectedPointcloud() const {
    return !config_.reprojected_pointcloud_topic_name.empty();
  }
  bool shouldPublishProjectedRangeImage() const {
    return !config_.projected_range_image_topic_name.empty();
  }

 protected:
  const InputHandlerConfig config_;
  const std::string world_frame_;

  std::vector<IntegratorBase::Ptr> integrators_;
  Timer integration_timer_;

  std::shared_ptr<TfTransformer> transformer_;

  virtual void processQueue() = 0;
  rclcpp::TimerBase::SharedPtr queue_processing_retry_timer_;

  void publishReprojectedPointcloud(const rclcpp::Time& stamp,
                                    const PosedPointcloud<>& posed_pointcloud);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      reprojected_pointcloud_pub_;

  void publishProjectedRangeImage(const rclcpp::Time& stamp,
                                  const Image<>& range_image);
  image_transport::Publisher projected_range_image_pub_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
