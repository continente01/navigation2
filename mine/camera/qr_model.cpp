


#include <math.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

QrModel::QrModel(
  //geometry_msgs::msg::Transform * camera_to_qr_transform, 
  map_t * map)      
: Camera(map) 
{
  //paramatri caratteristici del qr code necessari
} 

// Determine the probability for the given pose
double
QrModel::sensorFunction(CameraData * data, pf_sample_set_t * set)
{
  //determinare che tipo di funzione usare
  //deve ritornare il peso totale del set

  //dei qr code basta x,y,yaw -> pf_vector_t 
  //da definire i parametri qr_positions[]
  //però ora tratto il singolo qr_position
  pf_sample_t * sample;
  pf_vector_t pose;
  pf_vector_t camera_position;
  pf_vector_t base_position;
  double total_weight;
  total_weight = 0.0;

  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // si potrebbe fare direttamente  tf_buffer_.lookupTransform(qr_frame_id, base_frame_id, tf2::TimePoint())
    //ma dato che non conosco se ciò è possibile se il qr code esce dall'immagine, appena viene rilevato
    //catturo la trasformata e la salvo
    camera_position[0] = data.camera_to_qr_transform.translation.x+qr_position[0];
    camera_position[1] = data.camera_to_qr_transform.translation.y+qr_position[1];
    camera_position[2] = tf2::getYaw(data.camera_to_qr_transform.orientation)+qr_position[2];
  
    try {
       geometry_msgs::msg::TransformStamped camera_base_transform =
             tf_buffer_.lookupTransform(camera_frame_id, base_frame_id, tf2::TimePoint());
    } catch (tf2::TransformException &ex) {
       RCLCPP_ERROR(this->get_logger(), "Eccezione durante la ricerca della trasformata: %s", ex.what());
    }
    base_position[0] += camera_position[0]+camera_base_transform.translation.x;
    base_position[1] += camera_position[1]+camera_base_transform.translation.y;
    base_position[2] += camera_position[2]+tf2::getYaw(camera_base_transform.orientation);

    //parte brutta del calcolo del peso del sample o della media e varianza, forse meglio?
    //sezione method description https://www.mdpi.com/1424-8220/20/11/3145

  }


 return total_weight;
}


bool
BeamModel::sensorUpdate(pf_t * pf, CameraData * data)
{
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}
}