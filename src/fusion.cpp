#include "fusion.h"
#include <iostream>

using namespace carnd_ekf;

Fusion::Fusion() : is_init_(false), previous_timestamp_(0) {
  // initialize kalman filter variables

  // state X = [px, vx, py, vy]
  x_ = Eigen::VectorXd(4);

  // initial covariance P (pos=1, vel=1000)
  Eigen::VectorXd initial_var(4); initial_var << 1., 1000., 1., 1000.;
  P_ = initial_var.asDiagonal();

  // state transition matrix F (check update_process_model)
  F_ = Eigen::MatrixXd(4, 4);

  // process noise Q (check update_process_noise)
  Q_ = Eigen::MatrixXd(4, 4);

  // laser measurement = [px, py]
  // measurement noise
  R_laser_ = Eigen::MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  // measurement function
  H_laser_ = Eigen::MatrixXd(2, 4); // 2 measurements, 4 states
  H_laser_ << 1, 0, 0, 0,
              0, 0, 1, 0;

  // radar measurement = [rho, phi, rhodot]
  R_radar_ = Eigen::MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  // measurement function (calculated with Jacobian)
  H_radar_ = Eigen::MatrixXd(3, 4); // 3 measurements, 4 states
}

void Fusion::process_measurement(const MeasurementPackage &measurement_pack) {
  // initialize the kf
  if(!is_init_) {
    // first measurement: initialize x, P
    if(measurement_pack.sensor_type == MeasurementPackage::LIDAR) {
      // set initial state using lidar values (px, py)
      x_ << measurement_pack.raw_measurements(0), 0,
            measurement_pack.raw_measurements(1), 0;

      std::cout << "Initialized with LIDAR message:\n" << x_ << std::endl;
    } else if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {
      // set initial state using radar values (convert ρ,φ -> px, py)
      float rho = measurement_pack.raw_measurements(0);
      float phi = measurement_pack.raw_measurements(1);

      x_ << std::cos(phi) * rho, 0, // px, vx
            std::sin(phi) * rho, 0; // py, vy

      std::cout << "Initialized with RADAR message:\n" << x_ << std::endl;
    }

    previous_timestamp_ = measurement_pack.timestamp;
    is_init_ = true;
    return;
  }

  double dt = (measurement_pack.timestamp - previous_timestamp_) / 1000000.0;

  // prediction
  update_process_model(dt);
  update_process_noise(dt);
  predict();

  // update
  if(measurement_pack.sensor_type == MeasurementPackage::LIDAR) {
    R_ = R_laser_;
    H_ = H_laser_;
    update(measurement_pack.raw_measurements);
  } else if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {
    R_ = R_radar_;
    H_ = calculate_jacobian(x_);
    update(measurement_pack.raw_measurements,
           std::bind(&Fusion::measurement_function, this, std::placeholders::_1));
  }

  previous_timestamp_ = measurement_pack.timestamp;
}

void Fusion::update_process_model(const double dt) {
  // constant velocity model
  // x = x0 + delta_t * vx
  F_ << 1, dt, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt,
        0, 0, 0, 1;
}

void Fusion::update_process_noise(const double dt) {
  /* model process noise as by modelling acceleration
   * px = px + delta_t * vx + ax * Δt^2 / 2
   * vx = vx + ax * Δt
   * py = py + delta_t + vy + ay * Δt^2 / 2
   * vy = vy + ay * Δt
   *
   * random acceleration vector
   * ν = [ax * Δt^2 / 2; ax * Δt; ay * Δt^2 / 2; ay * Δt]
   * split ν
   * ν = [Δt^2 / 2, 0; Δt, 0; 0, Δt^2 / 2; 0, Δt] * [ax; ay] = Ga
   * Q = E[νν'] = E[Gaa'G'] = G E[aa'] G' = Q_v
   * Q_v = [var_ax covar_axy; covar_axy var_ay] (covar_axy=0)
   */
   double noise_ax = 9.0, noise_ay = 9.0;
   double dt_2 = dt * dt; // dt^2
   double dt_3 = dt * dt_2; // dt^3
   double dt_4 = dt * dt_3; // dt^4
   double dt_3_2 = dt_3 / 2.; // dt^3/2
   double dt_4_4 = dt_4 / 4.; // dt^4/4

   Q_ << dt_4_4 * noise_ax, dt_3_2 * noise_ax, 0, 0,
         dt_3_2 * noise_ax, dt_2 * noise_ax, 0, 0,
         0, 0, dt_4_4 * noise_ay, dt_3_2 * noise_ay,
         0, 0, dt_3_2 * noise_ay, dt_2 * noise_ay;
}

Eigen::VectorXd Fusion::get_state() {
  return x_;
}

Eigen::VectorXd Fusion::measurement_function(const Eigen::VectorXd& x) {
  Eigen::VectorXd h(3);
  float px = x(0);
  float vx = x(1);
  float py = x(2);
  float vy = x(3);

  float c1 = sqrt(px*px+py*py);
  h << c1,                  // ρ
       std::atan2(py, px),  // ϕ
       (px*vx + py*vy)/c1;  // ρ_dot
  return h;
}
