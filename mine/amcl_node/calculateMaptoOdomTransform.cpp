//UNICA MODIFICA IL CAMBIO DEL TIPO DEL PARAMETRO PASSATO ALLA FUNZIONE
//IN MODO CHE SI PIÙ GENERALE E NON SPECIFICO PER IL SENSORE


//hpp 

  /*
   * @brief Determine TF transformation from map to odom
   */
void calculateMaptoOdomTransform(
    const rclcpp::Time & sensor_timestamp,,
    const std::vector<amcl_hyp_t> & hyps,
    const int & max_weight_hyp);


//cpp

void
AmclNode::calculateMaptoOdomTransform(
  const rclcpp::Time & sensor_timestamp,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // subtracting base to odom from map to base and send map to odom instead
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
    tf2::Transform tmp_tf(q, tf2::Vector3(
        hyps[max_weight_hyp].pf_pose_mean.v[0],
        hyps[max_weight_hyp].pf_pose_mean.v[1],
        0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = sensor_timestamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform: (%s)", e.what());
    return;
  }

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  latest_tf_valid_ = true;
}