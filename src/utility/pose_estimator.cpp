#include "pose_estimator.hpp"
#include "math_const.hpp"

PoseEstimator::PoseEstimator() { 
    kalman_.InitState(Vector3d(0.0, 0.0, 0.0));
    kalman_.SetStateEquation(RpyFromGyro);  // enable non linear kalman filter
    kalman_.SetObserveEquation(Identity3d(), Zero3d());
    Matrix3d Q = Identity3d() * DEG_TO_RAD * DEG_TO_RAD; // sigma = 1 deg^2
    Matrix3d R = Identity3d() * 1e-6; // sigma = 1 deg^2
    kalman_.SetVariance(Q, R);
    calibration_ = false;
    counter_ = 0;
    omega_bias_ = Vector3d(0.0, 0.0, 0.0);
}

void PoseEstimator::StartCalibration() {
    if (calibration_ == false) {
        calibration_ = true;
        counter_ = 0;
        omega_bias_ = Vector3d(0.0, 0.0, 0.0);
        rpy_observe_mean_ = Vector3d(0.0, 0.0, 0.0);
        rpy_observe_var_ = Vector3d(0.0, 0.0, 0.0);
    }
}

Vector3d PoseEstimator::UpdateRpy(const Vector3d& alpha, const Vector3d& omega, double dt) {
    // estimate pose
    const Vector3d omega_dt = (omega - omega_bias_) * dt;
    const Vector3d rpy_observe = RpyFromAccel(alpha);
    const Vector3d rpy_estimate = kalman_.Apply(omega_dt, rpy_observe);

    // calibrate gyro bias
    if (calibration_) {
        omega_bias_ = (omega_bias_ * counter_ + omega) / (counter_ + 1);
        Calibration(rpy_observe);
    }

    return rpy_estimate;
}

Vector3d PoseEstimator::RpyFromAccel(const Vector3d& alpha) {
    const Vector3d alpha_0 = Vector3d(0.0, 0.0, -sqrt(alpha(X) * alpha(X) + alpha(Y) * alpha(Y) + alpha(Z) * alpha(Z)));
    return Rpy(alpha_0, alpha); // rotate coordinate
}

Vector3d PoseEstimator::RpyFromGyro(const Vector3d& rpy, const Vector3d& omega_dt) {
    return rpy + MatrixFromRpy(rpy) * omega_dt;
}

void PoseEstimator::Calibration(const Vector3d& rpy_observe) {
    // prepare for updating rpy observe
    const Vector3d rpy_observe_mean_next = (rpy_observe_mean_ * counter_ + rpy_observe) / (counter_ + 1);
    const Vector3d rpy_observe_mean_2 = Vector3d(rpy_observe_mean_(X) * rpy_observe_mean_(X), 
                                                    rpy_observe_mean_(Y) * rpy_observe_mean_(Y), 
                                                    rpy_observe_mean_(Z) * rpy_observe_mean_(Z));
    const Vector3d rpy_observe_mean_next_2 = Vector3d(rpy_observe_mean_next(X) * rpy_observe_mean_next(X), 
                                                        rpy_observe_mean_next(Y) * rpy_observe_mean_next(Y), 
                                                        rpy_observe_mean_next(Z) * rpy_observe_mean_next(Z));
    const Vector3d rpy_observe_2 = Vector3d(rpy_observe(X) * rpy_observe(X), 
                                            rpy_observe(Y) * rpy_observe(Y), 
                                            rpy_observe(Z) * rpy_observe(Z));
    const Vector3d rpy_observe_var_next = ((rpy_observe_var_ + rpy_observe_mean_2) * counter_ + rpy_observe_2) / (counter_ + 1) - rpy_observe_mean_next_2;
    // update rpy observe -> R
    rpy_observe_mean_ = rpy_observe_mean_next;
    rpy_observe_var_ = rpy_observe_var_next;
    
    counter_++;
    if (counter_ > NUM_CALIBRATION) {
        // reset
        calibration_ = false;
        counter_ = 0;
        // set variance
        double observe_var = 0.0;
        for (int i = 0; i < XYZ; i++) {
            observe_var += rpy_observe_var_(i);
        }
        observe_var /= XYZ;
        Matrix3d Q = Identity3d() * DEG_TO_RAD * DEG_TO_RAD; // sigma = 1 deg^2
        Matrix3d R = Identity3d();
        for (int i = 0; i < XYZ; i++) {
            R(i, i) = observe_var;
        }
        kalman_.SetVariance(Q, R);
    }
}
