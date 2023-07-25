//hpp
rclcpp::Time last_qr_received_ts_;
bool camera_update;

void qrReceived(geometry_msgs::msg::TransformStamped::ConstSharedPtr qr_detection);







//cpp

void
AmclNode::qrReceived(geometry_msgs::msg::TransformStamped::ConstSharedPtr qr_detection)
{
  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if (!active_) {return;}
  if (!first_map_received_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(get_logger(), "Waiting for map....");
      last_time_printed_msg_ = now();
    }
    return;
  }
  std::string qr_detection_frame_id = nav2_util::strip_leading_slash(qr_detection->header.frame_id);
  last_qr_received_ts_ = now();

  //int qr_index = -1; //serve nel caso un nuovo venga ricevuto nelle callback(?)
  //allora non so, lo usa quando non conosco la trasformata, quindi da valutare se necessario
  //geometry_msgs::msg::PoseStamped laser_pose;

  // Do we have the base->base_laser Tx yet?
  //frame to laser è una mappa che al frame id assegna un laser index. non credo a me serva
  //dato che ho un solo frame di un'unica camera. diciamo gestisce l'aggiunta di nuove
  
  // if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
  //   if (!addNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
  //     return;  // could not find transform
  //   }
  // } else {
  //   // we have the laser pose, retrieve laser index
  //   laser_index = frame_to_laser_[laser_scan->header.frame_id];
  // }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (!getOdomPose(
      latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
      qr_detection->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  bool force_publication = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    pf_init_ = true;

    // for (unsigned int i = 0; i < lasers_update_.size(); i++) {
    //   lasers_update_[i] = true;
    // }
    camera_update_ =true;

    force_publication = true;
    resample_count_ = 0;
  }
  else {
    // Set the laser update flags
    if (shouldUpdateFilter(pose, delta)) { //non da cambiare
      // for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      //   lasers_update_[i] = true;
      // }
      camera_update_ =true;
    }
    if (camera_update_) {
      motion_model_->odometryUpdate(pf_, pose, delta);
    }
    force_update_ = false;
  }
  bool resampled = false;

  // If the robot has moved, update the filter
  if (camera_update_) {
    updateFilter(laser_index, laser_scan, pose); // da modificare 

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      pf_update_resample(pf_, reinterpret_cast<void *>(map_));
      resampled = true;
    }

    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);

    if (!force_update_) {
      publishParticleCloud(set);
    }
  }

  if (resampled || force_publication || !first_pose_sent_) {
    amcl_hyp_t max_weight_hyps;
    std::vector<amcl_hyp_t> hyps; //ipotesi
    int max_weight_hyp = -1;
    if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp)) { // non da modificare
      
      publishAmclPose(/*MODIFICATO*/qr_detection, /*OK*/hyps, /*OK*/max_weight_hyp); // la funzione non va cambiata ma va cambiato il tipo di laser_scan, che da solo il time stamp
      calculateMaptoOdomTransform(/*MODIFICATO*/laser_scan, /*OK*/hyps, /*OK*/max_weight_hyp); // idem come sopra

      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        auto stamp = tf2_ros::fromMsg(qr_detection->header.stamp);
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        sendMapToOdomTransform(transform_expiration);
        sent_first_transform_ = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    if (tf_broadcast_ == true) {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      tf2::TimePoint transform_expiration = tf2_ros::fromMsg(qr_detecton->header.stamp) +
        transform_tolerance_;
      sendMapToOdomTransform(transform_expiration);
    }
  }
}