#include "wavemap_ros/input_handler/pointcloud_input_handler.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <wavemap/integrator/projective/projective_integrator.h>

namespace wavemap {
PointcloudInputHandler::PointcloudInputHandler(
    const InputHandlerConfig& config, const param::Map& params,
    std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, rclcpp::Node::SharedPtr nh)
    : InputHandler(config, params, std::move(world_frame),
                   std::move(occupancy_map), std::move(transformer), nh) {
  // Subscribe to the pointcloud input
  pointcloud_sub_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      config_.topic_name, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudInputHandler::pointcloudCallback, this,
                std::placeholders::_1));
}

void PointcloudInputHandler::processQueue() {
  while (!pointcloud_queue_.empty()) {
    const sensor_msgs::msg::PointCloud2& oldest_msg = pointcloud_queue_.front();
    const std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                            ? oldest_msg.header.frame_id
                                            : config_.sensor_frame_id;

    // Get the sensor pose in world frame
    Transformation3D T_W_C;
    if (!transformer_->lookupTransform(world_frame_, sensor_frame_id,
                                       oldest_msg.header.stamp, T_W_C)) {
      const auto newest_msg = pointcloud_queue_.back();
      if ((rclcpp::Time(newest_msg.header.stamp).seconds() - rclcpp::Time(oldest_msg.header.stamp).seconds()) <
          config_.max_wait_for_pose) {
        // Try to get this pointcloud's pose again at the next iteration
        return;
      } else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Waited " << config_.max_wait_for_pose
                                  << "s but still could not look up pose for "
                                     "pointcloud with frame \""
                                  << sensor_frame_id << "\" in world frame \""
                                  << world_frame_ << "\" at timestamp "
                                  << rclcpp::Time(oldest_msg.header.stamp).seconds()
                                  << "; skipping pointcloud.");
        pointcloud_queue_.pop();
        continue;
      }
    }

    // Convert the scan to a pointcloud
    // Get the index of the x field, and assert that the y and z fields follow
    auto x_field_iter = std::find_if(
        oldest_msg.fields.cbegin(), oldest_msg.fields.cend(),
        [](const sensor_msgs::msg::PointField& field) { return field.name == "x"; });
    if (x_field_iter == oldest_msg.fields.end()) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Received pointcloud with missing field x");
      return;
    } else if ((++x_field_iter)->name != "y") {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Received pointcloud with missing or out-of-order field y");
      return;
    } else if ((++x_field_iter)->name != "z") {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Received pointcloud with missing or out-of-order field z");
      return;
    }
    // Load the points into our internal Pointcloud type
    const size_t num_rays = oldest_msg.width * oldest_msg.height;
    std::vector<Point3D> t_C_points;
    t_C_points.reserve(num_rays);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(oldest_msg, "x");
         it != it.end(); ++it) {
      t_C_points.emplace_back(it[0], it[1], it[2]);
    }
    const PosedPointcloud<> posed_pointcloud(T_W_C, t_C_points);

    // Integrate the pointcloud
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Inserting pointcloud with "
                    << t_C_points.size()
                    << " points. Remaining pointclouds in queue: "
                    << pointcloud_queue_.size() - 1 << ".");
    integration_timer_.start();
    for (const auto& integrator : integrators_) {
      integrator->integratePointcloud(posed_pointcloud);
    }
    integration_timer_.stop();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Integrated new pointcloud in "
                    << integration_timer_.getLastEpisodeWallTime()
                    << "s. Total integration time: "
                    << integration_timer_.getTotalWallTime() << "s.");

    // Publish debugging visualizations
    if (shouldPublishReprojectedPointcloud()) {
      publishReprojectedPointcloud(oldest_msg.header.stamp, posed_pointcloud);
    }
    if (shouldPublishProjectedRangeImage()) {
      auto projective_integrator =
          std::dynamic_pointer_cast<ProjectiveIntegrator>(integrators_.front());
      if (projective_integrator) {
        const auto& range_image = projective_integrator->getPosedRangeImage();
        if (range_image) {
          publishProjectedRangeImage(oldest_msg.header.stamp, *range_image);
        }
      }
    }

    // Remove the pointcloud from the queue
    pointcloud_queue_.pop();
  }
}
}  // namespace wavemap
