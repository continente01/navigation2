//hpp


//cpp

nav2_util::CallbackReturn
AmclNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  executor_thread_.reset();

  // Get rid of the inputs first (services and message filter input), so we
  // don't continue to process incoming messages
  global_loc_srv_.reset();
  nomotion_update_srv_.reset();
  initial_pose_sub_.reset();
  laser_scan_connection_.disconnect();
  laser_scan_filter_.reset();
  laser_scan_sub_.reset();

  // Map
  if (map_ != NULL) {
    map_free(map_);
    map_ = nullptr;
  }
  first_map_received_ = false;
  free_space_indices.resize(0);

  // Transforms
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  // PubSub
  pose_pub_.reset();
  particle_cloud_pub_.reset();

  // Odometry
  motion_model_.reset();

  // Particle Filter
  pf_free(pf_);
  pf_ = nullptr;

  // Laser Scan
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
  force_update_ = true;
  
  // Qr Detection             //PARTE AGGIUNTA
  camera_.clear();
  camera_update_.clear();
  //frame_to_camera.clear()     // frame to laser serve ad associare il frame id con l'indice del laser, con una sola camera non serve
                              //FINE PARTE AGGIUNTA
  if (set_initial_pose_) {
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.x",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.x)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.y",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.y)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.z",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.z)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.yaw",
        rclcpp::ParameterValue(tf2::getYaw(last_published_pose_.pose.pose.orientation))));
  }

  return nav2_util::CallbackReturn::SUCCESS;
}