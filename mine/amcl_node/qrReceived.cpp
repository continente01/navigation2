//hpp
rclcpp::Time last_qr_received_ts_;
bool camera_update;

void qrReceived(geometry_msgs::msg::TransformStamped::ConstSharedPtr qr_detection);







//cpp

void
AmclNode::qrReceived(geometry_msgs::msg::TransformStamped::ConstSharedPtr qr_detection)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
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

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (!getOdomPose( //non da cambiare
      latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
      qr_detection->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  bool force_publication = false;
  if (!pf_init_) {              //se il pf non è inizializzato 
    // Pose at last filter update
    pf_odom_pose_ = pose;       //imposto la pf_odom_pose in pose(x,y,yam ottenuta da getOdomPose)

    pf_init_ = true;
    camera_update_ =true;
    force_publication = true;
    resample_count_ = 0;
  }
  else {                           //se il pf è già inizializzato
    // Set the camera update flag
    if (shouldUpdateFilter(pose, delta)) {  //controllo se il pf è da aggiornare, se lo è viene fatto in updateFilter
      camera_update_ =true;
      motion_model_->odometryUpdate(pf_, pose, delta);// 
    }
    force_update_ = false;
  }
  bool resampled = false;

  // If the robot has moved, update the filter
  if (camera_update_) {
    updateFilter(laser_index, qr_detection, pose); // da modificare 

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      pf_update_resample(pf_, reinterpret_cast<void *>(map_)); //non da modificare
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
      
      publishAmclPose(/*MODIFICATO*/qr_detection->header.stamp, /*OK*/hyps, /*OK*/max_weight_hyp); // la funzione non va cambiata ma va cambiato il tipo di laser_scan, che da solo il time stamp
      calculateMaptoOdomTransform(/*MODIFICATO*/qr_detection->header.stamp, /*OK*/hyps, /*OK*/max_weight_hyp); // idem come sopra

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