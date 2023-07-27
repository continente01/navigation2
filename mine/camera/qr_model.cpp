


#include <math.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

QrModel::QrModel(
  double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
  double lambda_short, double chi_outlier, size_t max_beams, map_t * map)      //capire che parametri sono necessari
: Camera(map) //trovare modo di convertire i beam in coordinate della camera
{
  z_hit_ = z_hit; // sono tutti parametri fissi informazioni per il robot
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  z_short_ = z_short;
  z_max_ = z_max;
  lambda_short_ = lambda_short;
  chi_outlier_ = chi_outlier;
} // probabilmente da aggiungere quelli del qr code

// Determine the probability for the given pose
double
QrModel::sensorFunction(CameraData * data, pf_sample_set_t * set)
{
  //determinare che tipo di funzione usare
  //deve ritornare il peso totale del set

  //dei qr code basta x,y,yaw -> pf_vector_t 
  //da definire i parametri qr_positions[]
  //pero ora tratto il singolo qr_position

  pf_vector_t base_position;
  base_position[0] = data.camera_to_qr_transform.translation.x+qr_position[0];
  base_position[1] = data.camera_to_qr_transform.translation.y+qr_position[1];
  base_position[2] = tf2::getYaw(data.camera_to_qr_transform.orientation)+qr_position[2];
  
  try {
      geometry_msgs::msg::TransformStamped camera_base_transform =
            tf_buffer_.lookupTransform(camera_frame_id, base_frame_id, tf2::TimePoint());
  } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Eccezione durante la ricerca della trasformata: %s", ex.what());
  }
  base_position[0] += camera_base_transform.translation.x;
  base_position[1] += camera_base_transform.translation.y;
  base_position[2] += tf2::getYaw(camera_base_transform.orientation);
  

  //calcolo dei pesi


  return 0; // MODIFICA 
}


bool
BeamModel::sensorUpdate(pf_t * pf, CameraData * data)
{
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}
}