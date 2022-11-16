#ifndef TRACKING_H_
#define TRACKING_H_

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

class Tracking{
  public:
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    void Initial_state();
    KalmanFilter kf_;
  private:
    bool is_initialized_;
    int64_t previous_timestamp_;

    // acceleration noise componets
    float noise_x;
    float noise_y;

};
#endif