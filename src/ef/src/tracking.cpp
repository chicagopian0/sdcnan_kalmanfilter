#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include "tracking.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tracking::Tracking(){
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // create one 4D state vector, we don't know yet the values 
    // of the state x
    kf_.x_ = VectorXd(4);

    // state covariance matrix P
    kf_.P_ = MatrixXd(4, 4);
    kf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    
    // measurement covariance
    kf_.R_ = MatrixXd(2, 2);
    kf_.R_ << 0.0225, 0,
              0, 0.0225;

    // measurement matrix
    kf_.H_ = MatrixXd(2, 4);
    kf_.H_ << 1, 0, 0, 0,
              0, 1, 0, 0;

    // the initial transition matrix
    kf_.F_ = MatrixXd(4, 4);
    kf_.F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
    
    // set the acceleration noise componets
    noise_x = 5;
    noise_y = 5;

}

Tracking::~Tracking(){ }

void Tracking::Initial_state(){
    cout << "Initial state: " << is_initialized_ << endl;
}

void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack){
    if (!is_initialized_){
        cout << "Kalman Filter Initialization" << endl;
        cout << "set state with initial location and zero velocity" << endl;
        kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;

        return;
    }
    // compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 100000.0;         // should be 1000000.0 not 1000000
    previous_timestamp_ = measurement_pack.timestamp_;

    
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // TODO: YOUR CODE HERE
	//1. Modify the F matrix so that the time is integrated
    kf_.F_(0, 2) = dt;
    kf_.F_(1, 3) = dt;

	//2. Set the process covariance matrix Q
    kf_.Q_ = MatrixXd(4, 4);            // forget setting Q_ row and col
    kf_.Q_ << 0.25*dt_4*noise_x,        0,                          0.5*dt_3*noise_x,           0,
              0,                        0.25*dt_4*noise_y,          0,                          0.5*dt_3*noise_y,
              0.5*dt_3*noise_x,         0,                          dt_2*noise_x,               0,
              0,                        0.5*dt_3*noise_y,           0,                          dt_2*noise_y;
    
    cout << " CURRENT: " << kf_.Q_ << endl;
/*     kf_.Q_ << 0.25*noise_x*pow(dt,4),           0,          0.5*noise_x*pow(dt,3),              0,

              0,            0.25*noise_y*pow(dt,4),             0,          0.5*noise_y*pow(dt,3),

              0.5*noise_x*pow(dt,3),            0,      noise_x*pow(dt, 2),                     0,

              0,            0.5*noise_y*pow(dt,3),          0,                  noise_y*pow(dt,2); */

	//3. Call the Kalman Filter predict() function
    kf_.Predict();
	//4. Call the Kalman Filter update() function
    kf_.Update(measurement_pack.raw_measurements_);
	

    
    // with the most recent raw measurements_
	std::cout << "x_= " << kf_.x_ << std::endl;
	std::cout << "P_= " << kf_.P_ << std::endl;
    std::cout << "--------------" << std::endl;;
}