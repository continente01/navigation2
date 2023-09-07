


#include <cmath>
#include <assert.h>
#include <iostream>
#include <Eigen/Dense>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

QrModel::QrModel(
  //geometry_msgs::msg::Transform * camera_to_qr_transform, 
  map_t * map)      
: Camera(map) 
{
  //paramatri caratteristici del qr code necessari probabilmente aggiungere un id e lista di parametri
} 

// Determine the probability for the given pose
double
QrModel::sensorFunction(CameraData * data, pf_sample_set_t * set)
{
  //determinare che tipo di funzione usare
  //deve ritornare il peso totale del set

  
  //da definire i parametri qr_positions[]
  //però ora tratto il singolo qr_position
  pf_sample_t * sample;
  pf_vector_t sample_pose;
  pf_vector_t camera_position;
  pf_vector_t base_position;
  pf_matrix_t sample_covariance;
  double total_weight;
  total_weight = 0.0;

  /*utilizzo di eigen per i calcoli semplificati con le matrici*/
  Eigen::MatrixXd covariance_m(3,3);
  Eigen::VectorXd pose_v(3), sample_v(3);

  camera_position[0] = data.camera_to_qr_transform.translation.x+qr_position[0];
  camera_position[1] = data.camera_to_qr_transform.translation.y+qr_position[1];
  camera_position[2] = tf2::getYaw(data.camera_to_qr_transform.orientation)+qr_position[2];
  
  try {
    geometry_msgs::msg::TransformStamped camera_base_transform =
        tf_buffer_.lookupTransform(camera_frame_id, base_frame_id, tf2::TimePoint());
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Eccezione durante la ricerca della trasformata: %s", ex.what());
  }

  // per completezza

  base_position[0] += camera_position[0]+camera_base_transform.translation.x;
  base_position[1] += camera_position[1]+camera_base_transform.translation.y;
  base_position[2] += camera_position[2]+tf2::getYaw(camera_base_transform.orientation);

  for(int i=0; i<3;i++)
    pose_v(i)=base_position[i];

  for (int a = 0; a < set->sample_count; a++) { //iterazione dei sample
    sample = set->samples + a;
    sample_pose = sample->pose;
    sample_covariance = sample->cov;

    for(int i=0; i<3;i++)
      sample_v(i)=sample_pose[i];
    
    for(int i=0; i<3;i++){
      for(int j=0; j<3;j++)
        covariance_m(i)(j)=sample_covariance[i][j];
    }
    
    /* calcolo attraverso formula con matrice di covarianza presente in https://www.mdpi.com/1424-8220/20/11/3145 */

    double det = covariance_m.determinant();
    Eigen::VectorXd prodotto=(sample_v-pose_v).transpose()*covariance_m.inverse()*(sample_v-pose_v);
    double esponente= -prodotto(0)/2*;
    double x=1/(std::pow(2*pi,3/2)*std::sqrt(det));
    double e=std::exp(esponente);
    double d=x*e;

    sample->weight=d;
    total_weight += d;
  }
  

 return total_weight; //peso complessivo del set
}


bool
BeamModel::sensorUpdate(pf_t * pf, CameraData * data)
{
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}
}