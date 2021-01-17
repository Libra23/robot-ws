#ifndef _IMU_ESTIMATOR_H
#define _IMU_ESTIMATOR_H

#include "kalman_filter.hpp"

#define NUM_CALIBRATION 100

class PoseEstimator {
    public:
    PoseEstimator();
    void StartCalibration();
    Vector3d UpdateRpy(const Vector3d& alpha, const Vector3d& omega, double dt);
    private:
    KalmanFilter kalman_;
    bool calibration_;
    unsigned int counter_;
    Vector3d rpy_observe_mean_, rpy_observe_var_;
    Vector3d omega_bias_;
    Vector3d RpyFromAccel(const Vector3d& alpha);
    static Vector3d RpyFromGyro(const Vector3d& rpy, const Vector3d& omega_dt);
    void Calibration(const Vector3d& rpy_observe);
};

#endif
