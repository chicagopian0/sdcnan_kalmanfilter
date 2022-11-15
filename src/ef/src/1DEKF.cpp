#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace Eigen;

// Kalman Filter variables

VectorXd x;     // object state
MatrixXd P;     // object covariance matrix
VectorXd u;     // external motion
MatrixXd F;     // state transition matrix
MatrixXd H;     // measurement matrix
MatrixXd R;     // measurement covariance matrix
MatrixXd I;     // identity matrix
MatrixXd Q;     // process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);

void filter(VectorXd &x, MatrixXd &P){
    for (unsigned int n = 0; n < measurements.size(); n++){
        VectorXd z = measurements[n];
        MatrixXd Ht = H.transpose();
        MatrixXd S = H * P * Ht + R;
        MatrixXd Si = S.inverse();
        MatrixXd K = P * Ht * Si;
              
        // new state  
        x = x + K * (z - H * x);
        P = (I - K * H) * P;
        


        // prediction
        MatrixXd Ft = F.transpose();
        x = F * x + u;
        P = F * P * Ft + Q;

        /* Why do we not use the process noise in the state prediction function,
        even though the state transition equation has one? In other words, 
        why does the code set u << 0, 0 for the equation x = F * x + u? */

        cout << "x = " << endl << x << endl;
        cout << "P = " << endl << P << endl;
    }
}

int main(){
    // design the KF with 1D motion
    x = VectorXd(2);
    x << 0, 0;

    P = MatrixXd(2, 2);
    P << 1000, 0, 0, 1000;

    u = VectorXd(2);
    u << 0, 0;
    F = MatrixXd(2, 2);
    F << 1, 1, 0, 1;

    H = MatrixXd(1, 2);
    H << 1, 0;

    R = MatrixXd(1, 1);
    R << 1;
  
    I = MatrixXd::Identity(2, 2);
 
    Q = MatrixXd(2, 2);
    Q << 0, 0, 0, 0;


    VectorXd single_meas(1);
    single_meas << 1;
    measurements.push_back(single_meas);
    single_meas << 2;
    measurements.push_back(single_meas);
    single_meas << 3;
    measurements.push_back(single_meas);

    filter(x, P);
    
    return 0;
}