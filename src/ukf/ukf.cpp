#include <iostream>
#include "ukf.h"

UKF::UKF(){
    Init();
}

UKF::~UKF() { }

void UKF::Init() { }

void UKF::GenerateSigmaPoints(MatrixXd *Xsig_out){
    // set state dimension
    int n_x = 5;

    // define spreading parameter
    double lambda = 3 - n_x;

    // set example state
    VectorXd x = VectorXd(n_x);
    x <<    5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;
    
    // set example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P <<   0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    // create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

    // calculate square root of P
    MatrixXd A = P.llt().matrixL();

    // first col equal x
    Xsig.col(0) = x;

    //calculate remaining sigma points
    for (int i = 0; i < n_x; i++)
    {
        Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);
        Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);
    }

  //print result
  //std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  //write result
  *Xsig_out = Xsig;

}