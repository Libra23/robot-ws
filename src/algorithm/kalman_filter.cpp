#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(double num_state, double num_input, double num_output) : 
    x_(VectorXd::Zero(num_state)),
    x_odometry_(VectorXd::Zero(num_state)), 
    y_odometry_(VectorXd::Zero(num_output)),
    A_(MatrixXd::Identity(num_state, num_state)),
    B_(MatrixXd::Zero(num_state, num_input)),
    C_(MatrixXd::Zero(num_output, num_state)),
    D_(MatrixXd::Zero(num_output, num_input)),
    P_(MatrixXd::Zero(num_state, num_state)),
    Q_(MatrixXd::Zero(num_state, num_state)),
    R_(MatrixXd::Zero(num_output, num_output)),
    is_non_linear_state_eq_(false),
    is_non_linear_observe_eq_(false) {}

void KalmanFilter::SetState(const VectorXd& x) {
    x_ = x;
}

void KalmanFilter::SetStateEquation(const MatrixXd& A, const MatrixXd& B) {
    A_ = A;
    B_ = B;
}

void KalmanFilter::SetStateEquation(const std::function<VectorXd(const VectorXd&, const VectorXd&)>& eq) {
    is_non_linear_state_eq_ = true;
    non_linear_state_equation_ = eq;
}

void KalmanFilter::SetObserveEquation(const MatrixXd& C, const MatrixXd& D) {
    C_ = C;
    D_ = D;
}

void KalmanFilter::SetObserveEquation(const std::function<VectorXd(const VectorXd&, const VectorXd&)>& eq) {
    is_non_linear_observe_eq_ = true;
    non_linear_observe_equation_ = eq;
}

void KalmanFilter::SetVariance(const MatrixXd& Q, const MatrixXd& R) {
    Q_ = Q;
    R_ = R;
}

VectorXd KalmanFilter::Apply(const VectorXd& u, const VectorXd& y) {
    // update x
    if (is_non_linear_state_eq_) {
        A_ = UpdateJacobian(x_, u, non_linear_state_equation_);   // update A
        x_odometry_ = non_linear_state_equation_(x_, u);
    } else {
        x_odometry_ = A_ * x_ + B_ * u;
    }

    // update y
    if(is_non_linear_observe_eq_) {
        C_ = UpdateJacobian(x_odometry_, u, non_linear_observe_equation_);    // update C
        y_odometry_ = non_linear_observe_equation_(x_odometry_, u);
    }else {
        y_odometry_ = C_ * x_odometry_ + D_ * u;
    }

    const MatrixXd P_odometry = A_ * P_ * A_.transpose() + Q_;
    const MatrixXd K_kalman = P_odometry * C_.transpose() * (C_ * P_odometry * C_.transpose() + R_).inverse();

    x_ = x_odometry_ + K_kalman * (y - y_odometry_);
    P_ = (Identity3d() - K_kalman * C_) * P_odometry;

    return x_;
}

VectorXd KalmanFilter::GetOdometryState() const {
    return x_odometry_;
}

VectorXd KalmanFilter::GetOdometryOutput() const {
    return y_odometry_;
}

MatrixXd KalmanFilter::UpdateJacobian(const VectorXd& x, const VectorXd& u, const std::function<VectorXd(const VectorXd&, const VectorXd&)>& f) {
    MatrixXd jacobian = MatrixXd::Zero(f(x, u).size(), x.size());
    const double delta_e = 1e-5;
    for (int i = 0; i < x.size(); i++) {
        // +
        VectorXd x_p = x;
        x_p(i) += delta_e;
        const VectorXd y_p = f(x_p, u);
        // -
        VectorXd x_m = x;
        x_m(i) -= delta_e;
        const VectorXd y_m = f(x_m, u);
        // center diff
        const VectorXd d_y = (y_p - y_m) / (2 * delta_e);
        // set to jacobian
        jacobian.col(i) = d_y;
    }
    return jacobian;
}