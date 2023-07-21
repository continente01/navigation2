#include <sys/types.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "nav2_amcl/sensors/camera/camera.hpp"

namespace nav2_amcl
{
Camera::Camera(map_t * map)    //costruttore con inizializzazione delle variabili e assegnazione di map
: max_samples_(0), max_obs_(0), temp_obs_(NULL) // immagino siano valori per numero massimo di sample,
                                                //osservazioni e una matrice di osservazioni temporanee
{
  map_ = map;
}

Camera::~Camera() //distruttore
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k]; //elimina un vettore in quanto tempo_obs_ è una matrice
    }
    delete[] temp_obs_;
  }
}

void
Camera::reallocTempData(int new_max_samples, int new_max_obs) //DA CONTROLLARE FUNZIONAMENTO
{
  if (temp_obs_) { //elimina la matrice
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
  max_obs_ = new_max_obs;
  max_samples_ = fmax(max_samples_, new_max_samples); //sceglie il maggiore dei due

  temp_obs_ = new double *[max_samples_](); //rialloca la matrice max_samplesXmax_samples
  for (int k = 0; k < max_samples_; k++) {
    temp_obs_[k] = new double[max_obs_]();
  }
}
void
Camera::SetCameraPose(pf_vector_t & camera_pose)
{
  camera_pose_ = camera_pose;
}
}