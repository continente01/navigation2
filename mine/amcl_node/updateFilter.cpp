// hpp
nav2_amcl::Camera camera_;
/*
 * @brief Update the PF
 */
bool updateFilter(
    const /*apriltag_ros_msgs::msg::AprilTagDetectionArray*/ geometry_msgs::msg::TransformStamped &qr_detection,
    const pf_vector_t &pose);

// cpp
bool AmclNode::updateFilter(
    const  /*apriltag_ros_msgs::msg::AprilTagDetectionArray*/ geometry_msgs::msg::TransformStamped &qr_detection,
    const pf_vector_t &pose)
{
  nav2_amcl::CameraData cdata;
  cdata.camera_to_qr_transform = qr_detection;
  // cdata.camera = camera_index; //non necessario
  // lasers_ diventa camera_, unica, da cambiare vettore in elemento singolo

  camera_->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::CameraData *>(&cdata));
  camera_update_ = false;
  pf_odom_pose_ = pose;
  return true;
}