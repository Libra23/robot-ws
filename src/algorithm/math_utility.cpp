#include "math_utility.hpp"
#include "math_const.hpp"

Matrix3d MatrixFromAxisAndValue(const Vector3d& axis, double value) {
    // Rodrigues
    Matrix3d A;
    A <<       0, -axis(Z),  axis(Y),
         axis(Z),        0, -axis(X),
        -axis(Y),  axis(X),        0;
    const Matrix3d R = Matrix3d::Identity() + A * sin(value) + A * A * (1- cos(value));
    return R;
}

Matrix3d MatrixFromRpy(const Vector3d& rpy) {
    double cr = cos(rpy(ROLL));
    double cp = cos(rpy(PITCH));
    double cy = cos(rpy(YAW));
    double sr = sin(rpy(ROLL));
    double sp = sin(rpy(PITCH));
    double sy = sin(rpy(YAW));
    Matrix3d Rx, Ry, Rz;
    Rz << cy, -sy, 0,
          sy,  cy, 0,
           0,   0, 1;
    Ry << cp, 0, sp,
           0, 1,  0,
         -sp, 0, cp;
    Rx << 1,  0,   0,
          0, cr, -sr,
          0, sr,  cr;
    return Rz * Ry * Rx; 
}

Vector3d Rpy(const Matrix3d& mat) {
    Vector3d rpy;
    if (1 - 1e-7 <= mat(2, 0) && mat(2, 0) <= 1 + 1e-7) {
        rpy(ROLL) = 0;
        if (mat(2, 0) > 0) {
            rpy(PITCH) = -M_PI_2;
            rpy(YAW) = atan2(-mat(1, 2), mat(1, 1));
        } else {
            rpy(PITCH) = M_PI_2;
            rpy(YAW) = -atan2(-mat(1, 2), mat(1, 1));
        }
    } else {
        rpy(ROLL) = atan2(mat(2, 1), mat(2, 2));
        rpy(PITCH) = atan2(-mat(2, 0), sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2)));
        rpy(YAW) = atan2(mat(1, 0), mat(0, 0));
    }
    return rpy;
}

Vector3d Rpy(const Vector3d& v, const Vector3d& v_pre) {
    const Vector3d w = v_pre.cross(v); // rotate from v_pre to v
    if (w.norm() == 0.0) return Vector3d::Zero();
    // prepare for rodrigues
    const Vector3d n = w.normalized(); // rotate axis
    double theta = acos((v.transpose() * v_pre).norm() / (v.norm() * v_pre.norm())); // rotate value
    const Matrix3d R = MatrixFromAxisAndValue(n, theta);
    return Rpy(R);
}

Vector6d Differentiate(const Affine3d& trans, const Affine3d& trans_pre, double dt) {
    Vector6d twist = Vector6d::Zero();
    const Vector3d linear_velocity = (trans.translation() - trans_pre.translation()) / dt;
    const Vector3d angular_velocity = (Rpy(trans.rotation()) - Rpy(trans_pre.rotation())) / dt;
    twist.head(XYZ) = linear_velocity;
    twist.tail(RPY) = angular_velocity;
    return twist;
}

Vector3d Differentiate(const Vector3d& pos, const Vector3d& pos_pre, double dt) {
    return (pos - pos_pre) / dt;
}