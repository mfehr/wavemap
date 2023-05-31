#include "wavemap_ros/input_handler/input_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <wavemap/integrator/integrator_factory.h>
#include <wavemap/integrator/projective/projective_integrator.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(InputHandlerConfig,
                      (topic_name)
                      (topic_queue_length)
                      (processing_retry_period, SiUnit::kSeconds)
                      (max_wait_for_pose, SiUnit::kSeconds)
                      (sensor_frame_id)
                      (image_transport_hints)
                      (depth_scale_factor)
                      (time_delay, SiUnit::kSeconds)
                      (reprojected_pointcloud_topic_name)
                      (projected_range_image_topic_name));

bool InputHandlerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}

InputHandler::InputHandler(const InputHandlerConfig& config,
                           const param::Map& params, std::string world_frame,
                           VolumetricDataStructureBase::Ptr occupancy_map,
                           std::shared_ptr<TfTransformer> transformer,
                           rclcpp::Node::SharedPtr nh)
    : config_(config.checkValid()),
      world_frame_(std::move(world_frame)),
      transformer_(std::move(transformer)) {
  // Create the integrators
  CHECK(param::map::keyHoldsValue<param::Array>(params, "integrators"));
  const auto integrators =
      param::map::keyGetValue<param::Array>(params, "integrators");
  for (const auto& integrator_params : integrators) {
    CHECK(integrator_params.holds<param::Map>());
    auto integrator = IntegratorFactory::create(
        integrator_params.get<param::Map>(), occupancy_map,
        IntegratorType::kRayTracingIntegrator);
    CHECK_NOTNULL(integrator);
    integrators_.emplace_back(std::move(integrator));
  }

  // Start the queue processing retry timer
  queue_processing_retry_timer_ =
      nh->create_wall_timer(std::chrono::milliseconds(static_cast<int>(config_.processing_retry_period * 1000.0)),
                              [&]() { processQueue(); });

  // Advertise the reprojected pointcloud publisher if enabled
  if (!config_.reprojected_pointcloud_topic_name.empty()) {
    reprojected_pointcloud_pub_ =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.reprojected_pointcloud_topic_name,
            config_.topic_queue_length);
  }
  if (!config_.projected_range_image_topic_name.empty()) {
    image_transport::ImageTransport image_transport(nh);
    projected_range_image_pub_ = image_transport.advertise(
        config_.projected_range_image_topic_name, config_.topic_queue_length);
  }
}
/*
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

sensor_msgs::msg::PointCloud2 msg;

// Fill in the size of the cloud
msg.height = 480;
msg.width = 640;

// Create the modifier to setup the fields and memory
sensor_msgs::PointCloud2Modifier mod(msg);

// Set the fields that our cloud will have
mod.setPointCloud2FieldsByString(2, "xyz", "rgb");

// Set up memory for our points
mod.resize(msg.height * msg.width);

// Now create iterators for fields
sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
{
  *iter_x = 0.0;
  *iter_y = 0.0;
  *iter_z = 0.0;
  *iter_r = 0;
  *iter_g = 255;
  *iter_b = 0;
}


*/


void InputHandler::publishReprojectedPointcloud(
    const rclcpp::Time& stamp, const PosedPointcloud<>& posed_pointcloud) {
  sensor_msgs::msg::PointCloud2 pointcloud2_msg;
  pointcloud2_msg.header.stamp = stamp;
  pointcloud2_msg.header.frame_id = world_frame_;

  pointcloud2_msg.height = 1;
  pointcloud2_msg.width = posed_pointcloud.size();
  
  sensor_msgs::PointCloud2Modifier mod(pointcloud2_msg);
  mod.resize(pointcloud2_msg.height * pointcloud2_msg.width);

  sensor_msgs::PointCloud2Iterator<float_t> iter_x(pointcloud2_msg, "x");
  sensor_msgs::PointCloud2Iterator<float_t> iter_y(pointcloud2_msg, "y");
  sensor_msgs::PointCloud2Iterator<float_t> iter_z(pointcloud2_msg, "z");

  const int num_points = posed_pointcloud.size();
  auto all_points = posed_pointcloud.getPointsGlobal();

  
  for (int i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z){
    auto point = all_points[i];
    *iter_x = point.x();
    *iter_y = point.y();
    *iter_z = point.z();
  }

  reprojected_pointcloud_pub_->publish(pointcloud2_msg);
}

void InputHandler::publishProjectedRangeImage(const rclcpp::Time& stamp,
                                              const Image<>& range_image) {
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = stamp;
  cv_image.encoding = "32FC1";
  cv::eigen2cv(range_image.getData(), cv_image.image);

  sensor_msgs::msg::Image::SharedPtr image_msg;
  image_msg = cv_image.toImageMsg();

  projected_range_image_pub_.publish(image_msg);
}
}  // namespace wavemap
