#include <iostream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include "eigen3/Eigen/Dense"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

MatrixXd CalculateJacobian(const VectorXd &x_state);

int main(){
    VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    MatrixXd Hj = CalculateJacobian(x_predicted);

    cout << "Hj:" << endl << Hj << endl;

    return 0;
}



MatrixXd CalculateJacobian(const VectorXd &x_state){
    MatrixXd Hj(3, 4);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = c1 * c2;

    if (fabs(c1) < 0.0001){
        cout << "DIVISION BY ZERO" << endl;
        return Hj;
    }
    Hj << px/c2,    py/c2,      0,      0,
          -(py/c1), px/c1,      0,      0,
          py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;

    return Hj;
}