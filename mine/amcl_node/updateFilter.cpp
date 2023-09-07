// hpp
nav2_amcl::Camera camera_;
/*
 * @brief Update the PF
 */
bool updateFilter(
    const /*apriltag_ros_msgs::msg::AprilTagDetectionArray*/ geometry_msgs::msg::TransformStamped &qr_detection,
    const pf_vector_t &pose);

// la parte commentata è una variante in cui c'è una confidenza calcolata da apriltag ma c'è
// discrepanze tra il messaggio del topic e il messaggio descritto dalla documentazione, per sicurezza nonn viene utilizzato

// cpp
bool AmclNode::updateFilter(
    const  /*apriltag_ros_msgs::msg::AprilTagDetectionArray*/ geometry_msgs::msg::TransformStamped &qr_detection,
    const pf_vector_t &pose)
{
  nav2_amcl::CameraData cdata;
  cdata.camera_to_qr_transform = qr_detection-> transform;                                      //trasformata
  cdata.qr_frame_id = nav2_util::strip_leading_slash(qr_detection->child_frame_id);            //nome del qr code
  
  // cdata.camera = camera_index;                                              //non necessario
                                                                               // lasers_ diventa camera_, elemento singolo

  camera_->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::CameraData *>(&cdata));
  camera_update_ = false;
  pf_odom_pose_ = pose;
  return true;
}