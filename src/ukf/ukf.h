#ifndef UKF_H
#define UKF_H
#include <vector>
#include "eigen3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF{
  public:
    UKF();
    virtual ~UKF();

    void Init();

    void GenerateSigmaPoints(MatrixXd *Xsig_out);
    void AugmentedSigmaPoints(MatrixXd *Xsig_out);
    void SigmaPointPrediction(MatrixXd *Xsig_out);
    void PredictMeanAndCovariance(VectorXd *x_pred, MatrixXd *P_pred);
    void PredictRadarMeasurement(VectorXd *z_out, MatrixXd *S_out);
    void UpdateState(VectorXd *x_out, MatrixXd *P_out);
};

#endif