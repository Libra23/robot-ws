#include "math_utility.hpp"
#include "math_const.hpp"

Matrix3d MatrixFromAxisAndValue(const Vector3d& axis, double value) {
    // Rodrigues
    Matrix3d A;
    A <<       0, -axis(Z),  axis(Y),
         axis(Z),        0, -axis(X),
        -axis(Y),  axis(X),        0;
    const Matrix3d R = Identity3d() + A * sin(value) + A * A * (1- cos(value));
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
    const Vector3d w = Cross3d(v_pre, v); // rotate from v_pre to v
    const double w_length = Norm3d(w);
    if (w_length == 0.0) return Vector3d(0.0, 0.0, 0.0);
    // prepare for rodrigues
    const Vector3d n = w / w_length; // rotate axis
    double theta = acos(fabs(Dot(v, v_pre)) / (Norm3d(v) * Norm3d(v_pre))); // rotate value
    const Matrix3d R = MatrixFromAxisAndValue(n, theta);
    return Rpy(R);
}

double Dot(const Vector3d& v0, const Vector3d& v1) {
#ifdef USE_BLA
    return (~v0 * v1)(0, 0);
#else
    return v0.transpose() * v1;
#endif
}

Vector3d Cross3d(const Vector3d& v0, const Vector3d& v1) {
#ifdef USE_BLA
    const Vector3d v = Vector3d(v0(Y) * v1(Z) - v0(Z) * v1(Y),
                                v0(Z) * v1(X) - v0(X) * v1(Z),
                                v0(X) * v1(Y) - v0(Y) * v1(X));
    return v;
#else
    return v0.cross(v1);
#endif
}

double Norm3d(const Vector3d& v) {
#ifdef USE_BLA
    double length = 0.0;
    for (int i = 0; i < XYZ; i++) {
        length += v(i) * v(i);
    }
    return sqrt(length);
#else
    return v.norm();
#endif
}

Matrix3d Identity3d() {
    Matrix3d E;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            E(i, j) = (i == j) ? 1.0 : 0.0;
        }
    }
    return E;
}

Matrix3d Zero3d() {
    Matrix3d O;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            O(i, j) = 0.0;
        }
    }
    return O;
}

#ifdef USE_BLA
void PrintVector3d(const Vector3d& vec) {
    for (int i = 0; i < vec.Rows; i++) {
        Serial.print(vec(i)); Serial.print(" ");
    }
    Serial.println();
}

void PrintMatrix3d(const Matrix3d& mat) {
    for (int i = 0;i < mat.Rows; i++) {
        for (int j = 0; j < mat.Cols; j++) {
            Serial.print(mat(i, j)); Serial.print(" ");
        }
        Serial.println();
    }
}
#endif
