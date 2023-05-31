# Changes
 - [x] Ament-ification
 - [ ] Basic ROS2-ification

# Open Points

 - [ ] Fix wavemap-extras.cmake export in packages (currently copied)
 - [ ] Removed config conversion from wavemap_ros_conversions (needed!)
 - [ ] Removed unused performance_stats_pub_ (needed?)
 - [ ] Did not implement the ROS2 service calls
 - [ ] Removed nh_private (how is this concept correctly implemented in ROS2?)
 - [ ] Commented out the depth and livox input handler
 - [ ] Commented out rosbag_processor
 - [ ] rclcpp::Node is passed around as SharedPtr copies, probably not ideal
 - [ ] Removed checks for number of subscribers for publishers (how to do this in ROS2?)

 # Other Comments
  - units missing
    - wavemap_ros: WavemapServerConfig -> thresholding_period, pruning_period, visualization_period (seconds)
  - Weird detour to PointCloud2 in InputHandler publishReprojectedPointcloud via PointCloud type
  - Might need to cancel timers for clean shutdown?
  - missing tf2_ros package in package.xml?
  - had to import a voliro-version of minkindr_conversions (ROS2 compatible)
