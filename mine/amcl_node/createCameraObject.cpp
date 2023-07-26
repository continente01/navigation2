//hpp
nav2_amcl::Camera * createCameraObject();

//cpp

nav2_amcl::Camera *
AmclNode::createCameraObject()
{
  RCLCPP_INFO(get_logger(), "createCameraObject");
    
  return new nav2_amcl::QrModel(
    z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_,
      0.0, max_beams_, map_);
}