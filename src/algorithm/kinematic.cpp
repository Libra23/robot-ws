#include "kinematic.hpp"
#include "math_utility.hpp"

Kinematic::Kinematic() : w_(1000.0),
                         k_(0.01),
                         w_0_(1000.0),
                         k_0_(0.01){

}

void Kinematic::Config(const KinematicModel& model, int num_ik_max) {
    model_ = model;
    num_ik_max_ = num_ik_max;
}

void Kinematic::Forward(const Joint& q, const Affine3d& base_trans, Affine3d& tip_trans){
    tip_trans = base_trans;
    for (size_t i = 0; i < NUM_JOINT; i++) {
        Affine3d trans = CvtModelToTrans(model_.xyz[i], model_.axis[i], model_.type[i], q(i));
        tip_trans = tip_trans * trans;
    }
    Affine3d end_trans = CvtModelToTrans(model_.xyz.back(), model_.axis.back(), model_.type.back(), 0);
    tip_trans = tip_trans * end_trans;
}

bool Kinematic::Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const Joint& init_q, Joint& q){
    Affine3d tip_trans_d;
    Joint q_d = init_q;
    Forward(q_d, base_trans, tip_trans_d);
    
    // prepare
    Twist twist = Differentiate(tip_trans, tip_trans_d, 1.0);
    Jacobian j = GetJacobian(q_d, base_trans, tip_trans_d);
    #ifdef BLA_H
    w_ = sqrt((j * ~j).Det());
    #else
    w_ = sqrt((j * j.transpose()).determinant());
    #endif
    k_ = (w_ >= w_0_) ? k_0_ : k_0_ * (1 - w_ / w_0_) *  (1 - w_ / w_0_);
    
    int num_ik = 0;
    double pre_error = 10000.0;
    while (true) {
        //Jacobian j_inv = j.Inverse();
        Jacobian j_inv = SingularityLowSensitiveInverse(j);
        Joint dq = j_inv * twist;
        q_d += dq;
        
        Forward(q_d, base_trans, tip_trans_d);
        twist = Differentiate(tip_trans, tip_trans_d, 1.0);
        Joint delta_q = q_d - init_q;
        double error = 0.0;
        for (int j = 0; j < DOF; j++) {
            error += (twist(j) * twist(j) + k_ * delta_q(j) * delta_q(j));
        }
        error = sqrt(error);
        if ((fabs(error - pre_error) < 1e-6)) break;
        pre_error = error;
        
        j = GetJacobian(q_d, base_trans, tip_trans_d);
        
        num_ik++;
        if (num_ik > num_ik_max_) return false;
    }
    q = q_d;
    return true;
 }

Affine3d Kinematic::CvtModelToTrans(const std::array<double, 3>& xyz, const std::array<double, 3>& axis, JointType type, double q) {
    Affine3d trans;
    if (type == ROTATE) {
        trans.translation() << xyz[X], xyz[Y], xyz[Z];
        double c = cos(q);
        double s = sin(q);
        const std::array<double, 3>& n = axis;
        trans.linear() << n[0] * n[0] * (1 - c) + c, n[0] * n[1] * (1 - c) - n[2] * s, n[0] * n[2] * (1 - c) + n[1] * s,
                          n[0] * n[1] * (1 - c) + n[2] * s, n[1] * n[1] * (1 - c) + c, n[1] * n[2] * (1 - c) - n[0] * s,
                          n[0] * n[2] * (1 - c) - n[1] * s, n[1] * n[2] * (1 - c) + n[0] * s, n[2] * n[2] * (1 - c) + c;
    } else {
        trans.translation() << xyz[X], xyz[Y], xyz[Z];
    }
    return trans;
}

Jacobian Kinematic::GetJacobian(const Joint& q,const Affine3d& base_trans, const Affine3d& tip_trans) {
    Jacobian jacobian;
    const double delta_q = 1e-5;
    for (int i = 0; i < NUM_JOINT; i++) {
        Joint q_p = q;
        q_p(i) += delta_q;
        Affine3d tip_trans_p;
        Forward(q_p, base_trans, tip_trans_p);
        Joint q_m = q;
        q_m(i) -= delta_q;
        Affine3d tip_trans_m;
        Forward(q_m, base_trans, tip_trans_m);
        Twist twist = Differentiate(tip_trans_p, tip_trans_m, 2 * delta_q);
        for (int j = 0; j < DOF; j++) {
            jacobian(j, i) = twist(j);
        }
    }
    return jacobian;
}

Jacobian Kinematic::SingularityLowSensitiveInverse(const Jacobian& jacobian) {
    Jacobian jacobian_plus;
    #ifdef BLA_H
    const Jacobian& jacobian_t = ~jacobian;
    jacobian_plus = jacobian_t * (jacobian * jacobian_t + IdentityDoF() * k_).Inverse();
    #else
    const Jacobian& jacobian_t = jacobian.transpose();
    jacobian_plus = jacobian_t * (jacobian * jacobian_t + IdentityDoF() * k_).inverse();
    #endif

    return jacobian_plus;
}

double Kinematic::GetControllability() const {
    return w_;
}

double Kinematic::GetStablility() const {
    return k_;
}

void Kinematic::SetConstant(double w, double k) {
    w_0_ = w;
    k_0_ = k;
}

Twist Differentiate(const Affine3d& trans, const Affine3d& trans_pre, double dt) {
    Twist twist ;
    Vector3d velocity = (trans.translation() - trans_pre.translation()) / dt;
    Vector3d angular_velocity = (Rpy(trans.rotation()) - Rpy(trans_pre.rotation())) / dt;
    for (int i = 0; i < DOF; i++) {
        twist(i) = velocity(i);
        if (DOF > XYZ) {
            twist(i + XYZ) = angular_velocity(i + XYZ);
        }
    }
    return twist;
}

MatrixDoF IdentityDoF() {
    MatrixDoF E;
    for (int i = 0; i < DOF; i++) {
        for (int j = 0; j < DOF; j++) {
            E(i, j) = (i == j) ? 1.0 : 0.0;
        }
    }
    return E;
}
