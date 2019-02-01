#include "sigma_points.h"

using namespace carnd_ekf;

MerweScaledSigmaPoints::MerweScaledSigmaPoints(int n, double alpha, double beta, double kappa,
  std::function<Eigen::VectorXd (const Eigen::VectorXd&, const Eigen::VectorXd&)> subtract)
  : n_(n), alpha_(alpha), beta_(beta), kappa_(kappa), subtract_(subtract) {
  compute_weights();
}

const Eigen::VectorXd& MerweScaledSigmaPoints::get_Wm() {
  return Wm_;
}

const Eigen::VectorXd& MerweScaledSigmaPoints::get_Wc() {
  return Wc_;
}

Eigen::MatrixXd MerweScaledSigmaPoints::get_sigma_points(Eigen::VectorXd& x, Eigen::MatrixXd& P) {
  Eigen::MatrixXd sigma = Eigen::MatrixXd(n_, 2 * n_ + 1);  // each col per sigma
  sigma.col(0) = x; // first sigma is just mean
  Eigen::MatrixXd U = matrix_sqrt((n_ + lambda_) * P);
  for(int k=0; k<n_; k++) {
    if(subtract_ != nullptr) { // if special subtract function available
      sigma.col(k+1) = subtract_(x, -U.col(k));
      sigma.col(k+1+n_) = subtract_(x, U.col(k));
    } else {
      sigma.col(k+1) = x + U.col(k);
      sigma.col(k+1+n_) = x - U.col(k);
    }
  }
  return sigma;
}

void MerweScaledSigmaPoints::compute_weights() {
  lambda_ = alpha_ * alpha_ * (n_ + kappa_) - n_;
  double c = 0.5 / (n_ + lambda_);
  Wm_ = Eigen::VectorXd(2 * n_ + 1);
  Wm_.fill(c);
  Wc_ = Wm_;
  Wm_[0] = lambda_ / (n_ + lambda_);
  Wc_[0] = lambda_ / (n_ + lambda_) + (1. - alpha_ * alpha_ + beta_);
}

inline Eigen::MatrixXd MerweScaledSigmaPoints::matrix_sqrt(Eigen::MatrixXd M) {
  return M.llt().matrixL();
}
