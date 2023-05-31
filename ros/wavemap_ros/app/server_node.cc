#include <gflags/gflags.h>

#include "wavemap_ros/wavemap_server.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("wavemap_server", "wavemap_server");
  CHECK(nh);
  wavemap::WavemapServer wavemap_server(nh);

  rclcpp::spin(nh);
  return 0;
}
