#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter() {
    A_ = Identity3d();
    B_ = Identity3d();
    C_ = Identity3d();
    D_ = Identity3d();
    P_ = Zero3d();
    Q_ = Zero3d();
    R_ = Zero3d();
    x_ = Vector3d(0.0, 0.0, 0.0);
    non_linear_state_ = false;
    non_linear_observe_ = false;
}

void KalmanFilter::InitState(const Vector3d& x) {
    x_ = x;
}

void KalmanFilter::SetStateEquation(const Matrix3d& A, const Matrix3d& B) {
    A_ = A;
    B_ = B;
}

void KalmanFilter::SetStateEquation(const std::function<Vector3d(const Vector3d&, const Vector3d&)>& eq) {
    non_linear_state_ = true;
    non_linear_state_equation_ = eq;
}

void KalmanFilter::SetObserveEquation(const Matrix3d& C, const Matrix3d& D) {
    C_ = C;
    D_ = D;
}

void KalmanFilter::SetObserveEquation(const std::function<Vector3d(const Vector3d&, const Vector3d&)>& eq) {
    non_linear_observe_ = true;
    non_linear_observe_equation_ = eq;
}

void KalmanFilter::SetVariance(const Matrix3d& Q, const Matrix3d& R) {
    Q_ = Q;
    R_ = R;
}

Vector3d KalmanFilter::Apply(const Vector3d& u, const Vector3d& y) {
    // update x
    Vector3d x_odometry;
    if (non_linear_state_) {
        A_ = UpdateJacobian(x_, u, non_linear_state_equation_);   // update A
        x_odometry = non_linear_state_equation_(x_, u);
    } else {
        x_odometry = A_ * x_ + B_ * u;
    }

    // update y
    Vector3d y_odometry;
    if(non_linear_observe_) {
        C_ = UpdateJacobian(x_odometry, u, non_linear_observe_equation_);    // update C
        y_odometry = non_linear_observe_equation_(x_odometry, u);
    }else {
        y_odometry = C_ * x_odometry + D_ * u;
    }

    #ifdef USE_BLA
    const Matrix3d P_odometry = A_ * P_ * ~A_ + Q_;
    const Matrix3d K_kalman = P_odometry * ~C_ * (C_ * P_odometry * ~C_ + R_).Inverse();
    #else
    const Matrix3d P_odometry = A_ * P_ * A_.transpose() + Q_;
    const Matrix3d K_kalman = P_odometry * C_.transpose() * (C_ * P_odometry * C_.transpose() + R_).inverse();
    #endif
    
    x_ = x_odometry + K_kalman * (y - y_odometry);
    P_ = (Identity3d() - K_kalman * C_) * P_odometry;

    return x_;
}

Vector3d KalmanFilter::GetState() const {
    return x_;
}

Matrix3d KalmanFilter::UpdateJacobian(const Vector3d& x, const Vector3d& u, const std::function<Vector3d(const Vector3d&, const Vector3d&)>& f) {
    Matrix3d jacobian;
    const double delta_e = 1e-5;
    // update odometry jacobian A
    for (int i = 0; i < NUM_STATE; i++) {
        // +
        Vector3d x_p = x;
        x_p(i) += delta_e;
        const Vector3d y_p = f(x_p, u);
        // -
        Vector3d x_m = x;
        x_m(i) -= delta_e;
        const Vector3d y_m = f(x_m, u);
        // center diff
        const Vector3d d_y = (y_p - y_m) / (2 * delta_e);
        for (int j = 0; j < NUM_STATE; j++) {
            jacobian(j, i) = d_y(j);
        }
    }
    return jacobian;
}