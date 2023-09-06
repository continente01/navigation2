//hpp

std::unique_ptr<message_filters::Subscriber<geometry_msgs::msg::TransformStamped,
    rclcpp_lifecycle::LifecycleNode>> qr_detection_sub_;

std::unique_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::TransformStamped>> qr_detection_filter_;

message_filters::Connection qr_detection_connection_;

std::string qr_topic_{"tf"}; /*detections*/  // la parte commentata è una variante in cui c'è la covariance calcolata da apriltag ma c'è
                                              //discrepanze tra il messaggio del topic e il messaggio descritto dalla documentazione
                                                

//cpp

AmclNode::initMessageFilters()
{
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), scan_topic_, rmw_qos_profile_sensor_data, sub_opt);

  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10,
    get_node_logging_interface(),
    get_node_clock_interface(),
    transform_tolerance_);


  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(
      &AmclNode::laserReceived,
      this, std::placeholders::_1));


// PARTE AGGIUNTA

  qr_detection_sub_ = std::make_unique<message_filters::Subscriber<geometry_msgs::msg::TransformStamped/*apriltag_ros_msgs::msg::AprilTagDetectionArray*/,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), qr_topic_, rmw_qos_profile_sensor_data, sub_opt);

  qr_detection_filter_ = std::make_unique<tf2_ros::MessageFilter<geometry_msgs::msg::TransformStamped/*apriltag_ros_msgs::msg::AprilTagDetectionArray*/>>(
    *qr_detection_sub_, *tf_buffer_, odom_frame_id_, 10,
    get_node_logging_interface(),
    get_node_clock_interface(),
    transform_tolerance_);

  qr_detection_connection_ = qr_detection_filter_->registerCallback(
    std::bind(
      &AmclNode::qrReceived,
      this, std::placeholders::_1));
}