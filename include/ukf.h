#ifndef _UKF_FILTER_H_
#define _UKF_FILTER_H_

#include <functional>
#include <cmath>
#include "Eigen/Dense"
#include "sigma_points.h"

namespace carnd_ekf {
  class Ukf {
  public:
    Ukf(SigmaPoints& points);

    ~Ukf();

    void predict(double dt);

    // Eigen::VectorXd



  private:
  }; // class Ukf
} // namespace carnd_ekf

#endif
