#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <eigen3/Eigen/Dense>

class MeasurementPackage{
  public:
    int64_t timestamp_;

    enum SensorType {
        LASER, RADAR
    }sensor_type_;

    Eigen::VectorXd raw_measurements_;
};
#endif