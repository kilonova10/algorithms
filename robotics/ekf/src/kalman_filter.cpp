#include <math.h>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const float PI2 = 2*M_PI;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}


void KalmanFilter::Predict()
{
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::Update(const VectorXd &z)
{
  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd Si = (H_ * PHt + R_).inverse();

  // get the Kalman gain
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  // update state covariance
  P_ = (I-K*H_)*P_;
}

