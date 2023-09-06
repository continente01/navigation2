// hpp
nav2_amcl::Camera camera_;
/*
 * @brief Update the PF
 */
bool updateFilter(
    const geometry_msgs::msg::TransformStamped &laser_scan,
    const pf_vector_t &pose);

// cpp
bool AmclNode::updateFilter(
    const geometry_msgs::msg::TransformStamped &qr_detection,
    const pf_vector_t &pose)
{
  nav2_amcl::CameraData cdata;
  cdata.camera_to_qr_transform = qr_detection;
  // cdata.camera = camera_index; //non necessario
  // lasers_ diventa camera_, unica, da cambiare vettore in elemento singolo

  /*necessità: convertire qr_detection in posizione del robot
  qr_detection è la trasformata tra camera_frame_id (camera_link) e il frame del qr
  conosco la posizione del frame del qr, quindi devo prendere la posizione del qr e
  ottenere la posizione del robot (concentrandomi solo su x,y,yaw)

  da gestire a seconda del parametro*/

  camera_->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::CameraData *>(&cdata));
  camera_update_ = false;
  pf_odom_pose_ = pose;
  return true;
}