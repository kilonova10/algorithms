#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage {
public:
  long long timestamp_;

  enum SensorType{
    LIDAR,
    RADAR
  } sensor_type_;

  Eigen::VectorXd gt_values_;

};

#endif 