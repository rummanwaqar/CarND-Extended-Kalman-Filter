#ifndef _UKF_FILTER_H_
#define _UKF_FILTER_H_

#include <functional>
#include <cmath>
#include <tuple>
#include "Eigen/Dense"
#include "sigma_points.h"

namespace carnd_ekf {
  class Ukf {
  public:
    Ukf(int n,
      SigmaPoints&& points,
      std::function<Eigen::VectorXd (const Eigen::VectorXd&, double)> fx,
      std::function<Eigen::VectorXd (const Eigen::VectorXd&, const Eigen::VectorXd&)> residual_fn = nullptr,
      std::function<Eigen::VectorXd (const Eigen::MatrixXd&, const Eigen::VectorXd&)> mean_fn = nullptr);

    ~Ukf();

    void predict(double dt);

  private:
    /*
     * Computes unscented transform of a set of sigma points and weights.
     * returns the mean and covariance in a tuple.
     * param sigmas: ndarray, of size (n, 2n+1)
     *   2D array of sigma points.
     * param Wm : ndarray [# sigmas per dimension]
     *   Weights for the mean.
     * param Wc : ndarray [# sigmas per dimension]
     *   Weights for the covariance.
     */
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> unscented_transform(
      Eigen::MatrixXd& sigmas, Eigen::VectorXd Wm, Eigen::VectorXd Wc);

    // sigma points object
    SigmaPoints& points_;

    // process function
    std::function<Eigen::VectorXd (const Eigen::VectorXd&, double)> fx_;

    // subtract function that computes the difference between two states
    std::function<Eigen::VectorXd (const Eigen::VectorXd&, const Eigen::VectorXd&)> residual_fn_;

    // mean calculation from sigma (used if state has angles)
    std::function<Eigen::VectorXd (const Eigen::MatrixXd&, const Eigen::VectorXd&)> mean_fn_;

  protected:
    // state vector
    Eigen::VectorXd x_;

    // state covariance
    Eigen::MatrixXd P_;
  }; // class Ukf
} // namespace carnd_ekf

#endif
