#include <math.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{
QrModel::QrModel(
  map_t * map)      //capire che parametri sono necessari
}: Camera(map) //trovare modo di convertire i beam in coordinate della camera
{
    //assegnazione dei parametri aggiuntivi che serviranno
}

// Determine the probability for the given pose
double
QrModel::sensorFunction(CameraData * data, pf_sample_set_t * set)
{

}