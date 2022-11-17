#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd CalculateRMES(const vector<VectorXd> &estimations,
                   const vector<VectorXd> &ground_truth);


int main(){
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    VectorXd e(4);
    e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);

    VectorXd g(4);
    g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    cout << CalculateRMES(estimations, ground_truth) << endl;

    return 0;
}

VectorXd CalculateRMES(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth){
                        
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0){
            cout << "INVALID ESTIMATIONS OR GROUND_TRUTH  DATA: " << endl;
            return rmse;
    }

    for (unsigned int i = 0; i < estimations.size(); i++){
        VectorXd residuals = estimations[i] - ground_truth[i];
        residuals = residuals.array() * residuals.array();
        rmse += residuals;
    }


    // calculate the mean
    rmse = rmse / estimations.size();
    // calculate the squared root
    rmse = rmse.array().sqrt();
    // return the result
    return rmse;
}