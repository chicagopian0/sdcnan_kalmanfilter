#ifndef KALMAN_FILTER_H_ 
#define KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter{
  public:
    // state vector
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // state transistion matrix
    MatrixXd F_;

    // process covariance matrix
    MatrixXd Q_;

    // measurement matrix
    MatrixXd H_;

    // measurement covariance matrix
    MatrixXd R_;

    KalmanFilter();

    virtual ~KalmanFilter();

    void Predict();

    void Update(const VectorXd &z);
};

#endif