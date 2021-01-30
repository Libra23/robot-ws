#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "math_utility.hpp"
#include <functional>

class KalmanFilter {
    public:
    KalmanFilter(double num_state, double num_input, double num_output);
    void SetState(const VectorXd& x);
    void SetStateEquation(const MatrixXd& A, const MatrixXd& B);
    void SetObserveEquation(const MatrixXd& C, const MatrixXd& D);
    void SetStateEquation(const std::function<VectorXd(const VectorXd&, const VectorXd&)>& eq);
    void SetObserveEquation(const std::function<VectorXd(const VectorXd&, const VectorXd&)>& eq);
    void SetVariance(const MatrixXd& Q, const MatrixXd& R);
    VectorXd Apply(const VectorXd& u, const VectorXd& y);
    VectorXd GetOdometryState() const;
    VectorXd GetOdometryOutput() const;
    static MatrixXd UpdateJacobian(const VectorXd& x, const VectorXd& u, const std::function<VectorXd(const VectorXd&, const VectorXd&)>& f);

    private:
    VectorXd x_, x_odometry_, y_odometry_;
    MatrixXd A_, B_, C_, D_;
    MatrixXd P_, Q_, R_;
    bool is_non_linear_state_eq_, is_non_linear_observe_eq_;
    std::function<VectorXd(const VectorXd&, const VectorXd&)> non_linear_state_equation_;
    std::function<VectorXd(const VectorXd&, const VectorXd&)> non_linear_observe_equation_;
};

#endif