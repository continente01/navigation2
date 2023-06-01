#ifndef NAV2_AMCL__SENSORS__LASER__LASER_HPP_
#define NAV2_AMCL__SENSORS__LASER__LASER_HPP_

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
   * @brief Laser destructor
   */
  virtual ~Camera();

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  virtual bool sensorUpdate(pf_t * pf, LaserData * data) = 0;

  /*
   * @brief Set the laser pose from an update
   * @param laser_pose Pose of the laser
   */
  void SetLaserPose(pf_vector_t & laser_pose);

protected:

  double z_hit_;    //GUARDA A COSA SERVE
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
}



}