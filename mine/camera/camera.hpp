#ifndef NAV2_AMCL__SENSORS__CAMERA__CAMERA_HPP_
#define NAV2_AMCL__SENSORS__CAMERA__CAMERA_HPP_

#include <string>
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/map/map.hpp"

namespace nav2_amcl
{
class Camera
{
public:
  /**
   * @brief A Camera constructor
   * @param map Map pointer to use
   */
  Camera(map_t * map);

  /*
   * @brief Camera destructor
   */
  virtual ~Camera();

  /*
   * @brief Run a sensor update on camera
   * @param pf Particle filter to use
   * @param data camera data to use
   * @return if it was succesful
   */
  virtual bool sensorUpdate(pf_t * pf, CameraData * data) = 0;

  /*
   * @brief Set the camera  pose from an update
   * @param camera _pose Pose of the camera 
   */
  void SetCameraPose(pf_vector_t & camera_pose);

protected:
  //nel dubbio metti tutto public poi si vede
  double z_hit_;    //GUARDA A COSA SERVE vanno cambiati a seconda di quello che serve per la camera
  double z_rand_;
  double sigma_hit_;


  /*
   * @brief Reallocate weights
   * @param max_samples Max number of samples
   * @param max_obs number of observations
   */
  void reallocTempData(int max_samples, int max_obs);
  map_t * map_;
  pf_vector_t camera_pose_;
  int max_samples_;
  int max_obs_;
  double ** temp_obs_;
};

class CameraData
{
public:
  //Camera * camera;  //essendoi una sola camera non dovrebbe essere necessario

  /*
   * @brief CameraData constructor
   */
  CameraData() {qr_position=NULL;}
  /*
   * @brief CameraData destructor
   */
  virtual ~CameraData() {delete qr_position}

public:
  geometry_msgs::msg::Transform qr_position;
};

class QrModel : public Camera   //                           **SICURAMENTE DA TOGLIERE O MODIFICARE, CONTROLLA DOVE USATO**
{
public:
  /*
   * @brief QrModel constructor
   */
  QrModel(
    double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
    double lambda_short, double chi_outlier, size_t max_beams, map_t * map); //da aggiungere parametri a seconda di quelli necessari

  /*
   * @brief Run a sensor update on Camera
   * @param pf Particle filter to use
   * @param data Camera data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, CameraData * data);

private:
  static double sensorFunction(CameraData * data, pf_sample_set_t * set);
  double z_short_;
  double z_max_;
  double lambda_short_;
  double chi_outlier_;
};
}