#ifndef MATH_UTILITY_H
#define MATH_UTILITY_H

#include <Eigen/Dense>
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Affine3d Affine3d;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::MatrixXd MatrixXd;

Matrix3d MatrixFromAxisAndValue(const Vector3d& axis, double value);
Matrix3d MatrixFromRpy(const Vector3d& rpy);
Vector3d Rpy(const Matrix3d& m);
Vector3d Rpy(const Vector3d& v, const Vector3d& v_pre);
double Dot(const Vector3d& v0, const Vector3d& v1);
Vector3d Cross3d(const Vector3d& v0, const Vector3d& v1);
double Norm3d(const Vector3d& v);
Matrix3d Identity3d();
Matrix3d Zero3d();

#endif