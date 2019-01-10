#ifndef _EKF_FUSION_H_
#define _EKF_FUSION_H_

#include <cmath>
#include "Eigen/Dense"

#include "kalman_filter.h"
#include "measurement_package.h"

namespace carnd_ekf {
  class Fusion : KalmanFilter {
  public:
    /*
     * Constructor
     */
    Fusion();

    /*
     * Destructor
     */
    virtual ~Fusion() = default;

    /*
     * run a full kalman filter cycle (predict -> update) using sensor measurements
     */
     void process_measurement(const carnd_ekf::MeasurementPackage& measurement_pack);

     /*
      * returns state vector
      */
     Eigen::VectorXd get_state();

   private:
     /*
      * updates the prcoess model using delta_t
      * @param delta_t = time_k+1 - time_k
      */
     void update_process_model(const double dt);

     /*
      * updates the process noise using delta_t
      * @param delta_t = time_k+1 - time_k
      */
     void update_process_noise(const double dt);

     /*
      * measurement function for radar. Converts state to measurement space (ekf)
      * @param x is state
      */
     Eigen::VectorXd measurement_function(const Eigen::VectorXd& x);

     /*
      * calculates Jacobian for radar measurement to linearize measurement function
      * @param x_state current state to linearize around
      */
     Eigen::MatrixXd calculate_jacobian(const Eigen::VectorXd& x_state);

   private:
     // check if kf has been initialized with first measurement
     bool is_init_;

     // previous timestamp
     long long previous_timestamp_;

     // measurement noise
     Eigen::MatrixXd R_laser_;
     Eigen::MatrixXd R_radar_;

     // measurement function
     Eigen::MatrixXd H_laser_;
     Eigen::MatrixXd H_radar_; // should be Jacobian

     // RMSE Tools Tools
  }; // class Fusion
} // namespace carnd_ekf

#endif // _EKF_FUSION_H_
