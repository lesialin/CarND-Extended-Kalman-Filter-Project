#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  //predict state

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  //update the state by using Kalman Filter equations

  VectorXd y = z - H_ * x_;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd K = P_ * H_t * S.inverse();
  //new state
  x_ = x_ + (K * y);
  MatrixXd I =  MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  //calculate predicted raser measurement
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float pred_rho = sqrt(px * px + py * py);
  float pred_phi = atan2(py, px);
  float pred_rho_dot = (px * vx + py * vy) / pred_rho;  
  VectorXd pred_z(3);
  pred_z <<  pred_rho, pred_phi, pred_rho_dot;
  //update
  VectorXd y = z - pred_z;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd K = P_ * H_t * S.inverse();
  //make sure the error angle is betwenn -pi to pi
  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    }
    else {
      y(1) += M_PI;
    }
  }
  
  //new state
  x_ = x_ + (K * y);
  MatrixXd I =  MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;



}
