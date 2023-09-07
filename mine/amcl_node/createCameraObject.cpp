//CREAZIONE OGGETTO CAMERA

//hpp
nav2_amcl::Camera * createCameraObject();

//cpp

nav2_amcl::Camera *
AmclNode::createCameraObject()
{
  RCLCPP_INFO(get_logger(), "createCameraObject");
    
  return new nav2_amcl::QrModel(map_);
}