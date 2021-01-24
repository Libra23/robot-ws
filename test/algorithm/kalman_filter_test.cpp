#include <gtest/gtest.h>
#include "kalman_filter.hpp"
#include "math_const.hpp"

constexpr double TOLERANCE = 1e-6;

class KalmanFilterTest : public ::testing::Test {
    protected:
    KalmanFilterTest() {
        Matrix3d Q = Identity3d() * 1e-6;
        Matrix3d R = Identity3d() * 1e-6;
        filter_.SetVariance(Q, R);
    }
    KalmanFilter filter_;
};

///////////////////////////////////////
// Check linear kalman filter component
///////////////////////////////////////
TEST_F(KalmanFilterTest, CheckLinearEquation) {
    // init state and equation
    const Vector3d x = Vector3d::Random();
    filter_.InitState(x);
    const Matrix3d A = Matrix3d::Random();
    const Matrix3d B = Matrix3d::Random();
    const Matrix3d C = Matrix3d::Random();
    const Matrix3d D = Matrix3d::Random();
    filter_.SetStateEquation(A, B);
    filter_.SetObserveEquation(C, D);
    // update
    const Vector3d u = Vector3d::Random();
    const Vector3d y = Vector3d::Random();
    filter_.Apply(u, y);
    // expected value
    const Vector3d x_expect = A * x + B * u;
    const Vector3d y_expect = C * x_expect + D * u;
    // kalman class value
    const Vector3d x_kalman = filter_.GetOdometryState();
    const Vector3d y_kalman = filter_.GetOdometryOutput();
    // check
    for (int i = 0; i < NUM_STATE; i++) {
        EXPECT_NEAR(x_expect(i), x_kalman(i), TOLERANCE);
        EXPECT_NEAR(y_expect(i), y_kalman(i), TOLERANCE);
    }
}

///////////////////////////////////////////
// Check NON linear kalman filter component
///////////////////////////////////////////
Vector3d NonLinearStateEq(Vector3d x, Vector3d u) {
    Vector3d ret;
    for (int i = 0; i < NUM_STATE; i++) {
        ret(i) = pow(x(i), 3) + pow(x(i), 2) + x(i) + pow(u(i), 3); // y = x^3 + x^2 + x + u^3
    }
    return ret;
}

Matrix3d JacobianNonLinearStateEq(Vector3d x, Vector3d u) {
    Matrix3d ret;
    for (int i = 0; i < NUM_STATE; i++) {
        for (int j = 0; j < NUM_STATE; j++) {
            ret(i, j) = (i == j) ? 3 * pow(x(i), 2) + 2 * x(i) + 1 : 0; // y = 3 x^2 + 2 x + 1
        }
    }
    return ret;
}

Vector3d NonLinearObserveEq(Vector3d x, Vector3d u) {
    Vector3d ret;
    ret(X) = x(X) * x(Y) * x(Z) + u(X); // y_0 = x_0 * x_1 * x_2 + u_0
    ret(Y) = x(X) * x(Y) * x(Z) + u(Y); // y_1 = x_0 * x_1 * x_2 + u_1
    ret(Z) = x(X) * x(Y) * x(Z) + u(Z); // y_2 = x_0 * x_1 * x_2 + u_2
    return ret;
}

Matrix3d JacobianNonLinearObserveEq(Vector3d x, Vector3d u) {
    Matrix3d ret;
    for (int i = 0; i < NUM_STATE; i++) {
        ret(i, X) = x(Y) * x(Z);
        ret(i, Y) = x(X) * x(Z);
        ret(i, Z) = x(X) * x(Y);
    }
    return ret;
}

TEST_F(KalmanFilterTest, CheckNonLinearEquation) {
    const Vector3d x = Vector3d::Random();
    filter_.InitState(x);
    filter_.SetStateEquation(NonLinearStateEq);
    filter_.SetObserveEquation(NonLinearObserveEq);
    // update
    const Vector3d u = Vector3d::Random();
    const Vector3d y = Vector3d::Random();
    filter_.Apply(u, y);
    // expected
    const Vector3d x_expect = NonLinearStateEq(x, u);
    const Vector3d y_expect = NonLinearObserveEq(x_expect, u);
    // kalman class
    const Vector3d x_kalman = filter_.GetOdometryState();
    const Vector3d y_kalman = filter_.GetOdometryOutput();
    // check
    for (int i = 0; i < NUM_STATE; i++) {
        EXPECT_NEAR(x_expect(i), x_kalman(i), TOLERANCE);
        EXPECT_NEAR(y_expect(i), y_kalman(i), TOLERANCE);
    }
}

TEST_F(KalmanFilterTest, CheckJacobian) {
    const int num_test = 100;
    for (int n = 0; n < num_test; n++) {
        Vector3d x = Vector3d::Random();
        Vector3d u = Vector3d::Random();
        std::cout << " x " << x.transpose() << std::endl;
        std::cout << " u " << u.transpose() << std::endl;
        // true
        Matrix3d jacobian_state_expect = JacobianNonLinearStateEq(x, u);
        std::cout << " jacobian_state_expect " << jacobian_state_expect << std::endl;
        Matrix3d jacobian_observe_expect = JacobianNonLinearObserveEq(x, u);
        std::cout << " jacobian_observe_expect " << jacobian_observe_expect << std::endl;
        // kalman class
        Matrix3d jacobian_state_kalman = KalmanFilter::UpdateJacobian(x, u, NonLinearStateEq);
        std::cout << " jacobian_state_kalman " << jacobian_state_kalman << std::endl;
        Matrix3d jacobian_observe_kalman = KalmanFilter::UpdateJacobian(x, u, NonLinearObserveEq);
        std::cout << " jacobian_observe_kalman " << jacobian_observe_kalman << std::endl;
        // check
        for (int i = 0; i < NUM_STATE; i++) {
            for (int j = 0; j < NUM_STATE; j++) {
                EXPECT_NEAR(jacobian_state_expect(i, j), jacobian_state_kalman(i, j), TOLERANCE);
                EXPECT_NEAR(jacobian_observe_expect(i, j), jacobian_observe_kalman(i, j), TOLERANCE);
            }
        }
    }
}