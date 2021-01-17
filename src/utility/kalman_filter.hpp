#include "math_utility.hpp"
#include <functional>

#define NUM_STATE 3

class KalmanFilter {
    public:
    KalmanFilter();
    void InitState(const Vector3d& x);
    void SetStateEquation(const Matrix3d& A, const Matrix3d& B);
    void SetObserveEquation(const Matrix3d& C, const Matrix3d& D);
    void SetStateEquation(const std::function<Vector3d(const Vector3d&, const Vector3d&)>& eq);
    void SetObserveEquation(const std::function<Vector3d(const Vector3d&, const Vector3d&)>& eq);
    void SetVariance(const Matrix3d& Q, const Matrix3d& R);
    Vector3d Apply(const Vector3d& u, const Vector3d& y);
    Vector3d GetState() const;
    Matrix3d UpdateJacobian(const Vector3d& x, const Vector3d& u, const std::function<Vector3d(const Vector3d&, const Vector3d&)>& f);

    private:
    Vector3d x_;
    Matrix3d A_, B_, C_, D_;
    Matrix3d P_, Q_, R_;
    bool non_linear_state_, non_linear_observe_;
    std::function<Vector3d(const Vector3d&, const Vector3d&)> non_linear_state_equation_;
    std::function<Vector3d(const Vector3d&, const Vector3d&)> non_linear_observe_equation_;
};