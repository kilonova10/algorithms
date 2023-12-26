#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {

public: 
	FusionEKF();

	virtual ~FusionEKF();

	void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	KalmanFilter ekf_;
}