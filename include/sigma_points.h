#ifndef _UKF_SIGMA_H_
#define _UKF_SIGMA_H_

#include <functional>
#include <cmath>
#include "Eigen/Dense"

namespace carnd_ekf {
  /*
   * Generates sigma points and weights according to Van der Merwe's
   * 2004 dissertation for the UnscentedKalmanFilter class.
   */
  class MerweScaledSigmaPoints {
  public:
    /*
     * initializes sigma point weights
     * param n : int
     *  Dimensionality of the state. 2n+1 weights will be generated.
     * param alpha : double, default=1e-3
     *  Determines the spread of the sigma points around the mean.
     * param beta : double, default=2.0
     *  Incorporates prior knowledge of the distribution of the mean. For
     *  Gaussian x beta=2 is optimal.
     * kappa : float, default=0.0
     *  Secondary scaling parameter usually set to 0 according to [4],
     *  or to 3-n according to [5].
     */
    MerweScaledSigmaPoints(int n, double alpha = 1e-3, double beta = 2., double kappa = 0.,
      std::function<Eigen::VectorXd (const Eigen::VectorXd&, const Eigen::VectorXd&)> subtract = nullptr);

    /*
     * return reference to mean weights
     */
    const Eigen::VectorXd& get_Wm();

    /*
     * return reference to covariance weights
     */
    const Eigen::VectorXd& get_Wc();

    /*
     * Computes the sigma points for an unscented Kalman filter
     * given the mean (x) and covariance(P) of the filter.
     */
    Eigen::MatrixXd get_sigma_points(Eigen::VectorXd& x, Eigen::MatrixXd& P);

  private:
    /*
     * Computes the weights for the scaled unscented Kalman filter
     */
    void compute_weights();

    /*
     * calculates √M using standard cholesky decomposition
     */
    Eigen::MatrixXd matrix_sqrt(Eigen::MatrixXd M);

    // Dimensionality of state
    int n_;
    // parametizes the sigma points using alpha, beta, kappa
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;

    // weight for each sigma point for the mean
    Eigen::VectorXd Wm_;
    // weight for each sigma point for the covariance
    Eigen::VectorXd Wc_;

    // subtract function that computes the difference between x and y.
    std::function<Eigen::VectorXd (const Eigen::VectorXd&, const Eigen::VectorXd&)> subtract_;


  }; // class MerweScaledSigmaPoints
} // namespace carnd_ekf

#endif
