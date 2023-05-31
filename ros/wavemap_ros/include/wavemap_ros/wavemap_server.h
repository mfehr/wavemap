#ifndef WAVEMAP_ROS_WAVEMAP_SERVER_H_
#define WAVEMAP_ROS_WAVEMAP_SERVER_H_

#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <wavemap/common.h>
#include <wavemap/config/config_base.h>
#include <wavemap_msgs/msg/map.hpp>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

#include "wavemap/integrator/integrator_base.h"
#include "wavemap_ros/input_handler/input_handler.h"
#include "wavemap_ros/tf_transformer.h"
#include "wavemap_ros/utils/timer.h"

namespace wavemap {
struct WavemapServerConfig : ConfigBase<WavemapServerConfig, 4> {
  std::string world_frame = "odom";
  FloatingPoint thresholding_period = 1.f;
  FloatingPoint pruning_period = 10.f;
  FloatingPoint visualization_period = 10.f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class WavemapServer {
 public:
  WavemapServer(rclcpp::Node::SharedPtr nh);
  WavemapServer(rclcpp::Node::SharedPtr nh,
                const WavemapServerConfig& config);

  void visualizeMap(rclcpp::Node::SharedPtr nh);
  bool saveMap(const std::string& file_path) const;
  bool loadMap(const std::string& file_path);

  InputHandler* addInput(const param::Map& integrator_params,
                         rclcpp::Node::SharedPtr nh);

  VolumetricDataStructureBase::Ptr getMap() { return occupancy_map_; }
  VolumetricDataStructureBase::ConstPtr getMap() const {
    return occupancy_map_;
  }

 private:
  const WavemapServerConfig config_;

  VolumetricDataStructureBase::Ptr occupancy_map_;

  std::shared_ptr<TfTransformer> transformer_;
  std::vector<std::unique_ptr<InputHandler>> input_handlers_;

  void subscribeToTimers(rclcpp::Node::SharedPtr nh);
  
  rclcpp::TimerBase::SharedPtr map_pruning_timer_;
  rclcpp::TimerBase::SharedPtr map_thresholding_timer_;
  rclcpp::TimerBase::SharedPtr map_visualization_timer_;
  rclcpp::TimerBase::SharedPtr map_autosave_timer_;

  void subscribeToTopics(rclcpp::Node::SharedPtr nh);

  void advertiseTopics(rclcpp::Node::SharedPtr nh);
  
  rclcpp::Publisher<wavemap_msgs::msg::Map>::SharedPtr map_pub_;

  // void advertiseServices(rclcpp::Node::SharedPtr nh);

  // ros::ServiceServer visualize_map_srv_;
  // ros::ServiceServer save_map_srv_;
  // ros::ServiceServer load_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_WAVEMAP_SERVER_H_
