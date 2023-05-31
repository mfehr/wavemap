#include "wavemap_ros/wavemap_server.h"

// #include <std_srvs/Empty.h>
#include <wavemap_msgs/srv/file_path.hpp>

#include <wavemap/data_structure/volumetric/volumetric_data_structure_factory.h>
#include <wavemap/utils/nameof.h>
#include <wavemap_msgs/msg/map.hpp>
#include <wavemap_msgs/msg/map_evaluation_summary.hpp>
#include <wavemap_msgs/msg/performance_stats.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_ros/input_handler/input_handler_factory.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(WavemapServerConfig,
                      (world_frame)
                      (thresholding_period, SiUnit::kSeconds)
                      (pruning_period, SiUnit::kSeconds)
                      (visualization_period, SiUnit::kSeconds));

bool WavemapServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, std::string(""), verbose);

  return all_valid;
}

WavemapServer::WavemapServer(rclcpp::Node::SharedPtr nh)
    : WavemapServer(nh,
                    WavemapServerConfig()) {}

WavemapServer::WavemapServer(rclcpp::Node::SharedPtr nh,
                             const WavemapServerConfig& config)
    : config_(config.checkValid()),
      transformer_(std::make_shared<TfTransformer>(nh)) {
  // Setup data structure
  const param::Map data_structure_params;
  // TODO(mfehr): add params manually here.
  
  occupancy_map_ = VolumetricDataStructureFactory::create(
      data_structure_params, VolumetricDataStructureType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  // Setup input handlers
  const param::Array integrator_params_array;
  // TODO(mfehr): add params manually here.
  for (const auto& integrator_params : integrator_params_array) {
    if (integrator_params.holds<param::Map>()) {
      const auto param_map = integrator_params.get<param::Map>();
      addInput(param_map, nh);
    }
  }

  // Connect to ROS
  subscribeToTimers(nh);
  subscribeToTopics(nh);
  advertiseTopics(nh);

  // advertiseServices(nh);
}

void WavemapServer::visualizeMap(rclcpp::Node::SharedPtr nh) {
  if (occupancy_map_ && !occupancy_map_->empty()) {
    occupancy_map_->threshold();

    const wavemap_msgs::msg::Map map_msg =
        convert::mapToRosMsg(occupancy_map_, config_.world_frame,
                             nh->now(), config_.visualization_period);
    map_pub_->publish(map_msg);
  }
}

bool WavemapServer::saveMap(const std::string& file_path) const {
  LOG(ERROR) << "Could not save map to " << file_path
             << ". Map saving not yet implemented.";
  return false;
}

bool WavemapServer::loadMap(const std::string& file_path) {
  LOG(ERROR) << "Could not load map from " << file_path
             << ". Map loading not yet implemented.";
  return false;
}

InputHandler* WavemapServer::addInput(const param::Map& integrator_params,
                                      rclcpp::Node::SharedPtr nh) {
  auto input_handler =
      InputHandlerFactory::create(integrator_params, config_.world_frame,
                                  occupancy_map_, transformer_, nh);
  if (input_handler) {
    return input_handlers_.emplace_back(std::move(input_handler)).get();
  }
  return nullptr;
}

void WavemapServer::subscribeToTimers(rclcpp::Node::SharedPtr nh) {
  if (0.f < config_.thresholding_period) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Registering map thresholding timer with period "
                           << config_.thresholding_period << "s");
    map_thresholding_timer_ =
        nh->create_wall_timer(std::chrono::milliseconds(static_cast<int>(config_.thresholding_period * 1000)),
                              [&]() { occupancy_map_->threshold(); });
  }

  if (0.f < config_.pruning_period) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Registering map pruning timer with period "
                           << config_.pruning_period << "s");
    map_pruning_timer_ =
        nh->create_wall_timer(std::chrono::milliseconds(static_cast<int>(config_.pruning_period * 1000)),
                              [&]() { occupancy_map_->prune(); });
  }

  if (0.f < config_.visualization_period) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Registering map visualization timer with period "
                           << config_.visualization_period << "s");
    map_visualization_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(config_.visualization_period * 1000)),
        [&]() { visualizeMap(nh); });
  }
}

void WavemapServer::subscribeToTopics(rclcpp::Node::SharedPtr /*nh*/) {}

void WavemapServer::advertiseTopics(rclcpp::Node::SharedPtr nh) {
  map_pub_ = nh->create_publisher<wavemap_msgs::msg::Map>(
      "map", 10);
}

// void WavemapServer::advertiseServices(rclcpp::Node::SharedPtr nh) {
//   visualize_map_srv_ = nh->advertiseService<std_srvs::Empty::Request,
//                                                    std_srvs::Empty::Response>(
//       "visualize_map", [this](auto& /*request*/, auto& /*response*/) {
//         visualizeMap(nh);
//         return true;
//       });

//   save_map_srv_ =
//       nh->advertiseService<wavemap_msgs::msg::FilePath::Request,
//                                   wavemap_msgs::msg::FilePath::Response>(
//           "save_map", [this](auto& request, auto& response) {
//             response.success = saveMap(request.file_path);
//             return true;
//           });

//   load_map_srv_ =
//       nh->advertiseService<wavemap_msgs::msg::FilePath::Request,
//                                   wavemap_msgs::msg::FilePath::Response>(
//           "load_map", [this](auto& request, auto& response) {
//             response.success = loadMap(request.file_path);
//             return true;
//           });
// }
}  // namespace wavemap
