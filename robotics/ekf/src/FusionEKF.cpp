#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools tools;

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  H_radar_ = MatrixXd(3, 4);

  //measurement covariance matrix - lidar
  R_laser_ << 0.0125, 0,
              0, 0.0125;

  //measurement covariance matrix - radar
  R_radar_ << 0.07, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.07;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

	// initialize the kalman filter variables
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // set measurement noises
  noise_ax = 9;
  noise_ay = 9;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

   /*  Initialization*/
  if (!is_initialized_) {

    // first measurement
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      float rho = measurement_pack.raw_measurements_[0];      // range: radial distance from origin
      float phi = measurement_pack.raw_measurements_[1];      // bearing: angle between rho and x axis
      float rho_dot = measurement_pack.raw_measurements_[2];  // radial velocity: change of rho

      ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;  // x, y, vx, vy


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR) {
  
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0; // x, y, vx, vy
    }


    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*  Prediction */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //  in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
             0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
             dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
             0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;

  ekf_.Predict();

   /*  Update*/


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    H_radar_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = H_radar_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

}