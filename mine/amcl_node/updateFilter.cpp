//hpp
nav2_amcl::Camera camera_;
  /*
   * @brief Update the PF
   */
bool updateFilter(
    const geometry_msgs::msg::TransformStamped & laser_scan,
    const pf_vector_t & pose);



//cpp
bool AmclNode::updateFilter(
  const geometry_msgs::msg::TransformStamped & qr_detection,
  const pf_vector_t & pose)
{
  nav2_amcl::CameraData cdata;
  cdata.qr_position=qr_detection;
  //cdata.camera = camera_index; //non necessario teoricamente

  //conversione frame qr -> camera -> base -> (mappa?)
  //il problema nasce dal fatto che io so come cambiare frame ma non so nè dove il frame è definito nè come capisco dove è la base

    //lasers_ diventa camera_, unica, da cambiare vettore in elemento singolo
  //lasers_[laser_index]->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::LaserData *>(&ldata)); // da modificare
  
  //necessità, convertire qr_detection in posizione del robot
  //qr_detection è la trasformata tra camera_frame_id e il frame del qr
  //conosco la posizione del frame del qr, quindi devo prendere la posizione del qr e 
  //ottenere la posizione del robot (concentrandomi solo su x,y,yaw)
  //non qui, da fare probabilmente in sensor update per la valutazione dei pesi

  camera_->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::CameraData *>(&cdata));
  camera_update_= false;
  pf_odom_pose_ = pose;
  return true;
}