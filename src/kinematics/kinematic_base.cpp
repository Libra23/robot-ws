#include "kinematic_base.hpp"

KinematicBase::KinematicBase() {}

KinematicBase::~KinematicBase() {

}

void KinematicBase::Config(const KinematicModel& model, uint8_t num_ik_max) {
    model_ = model;
    num_ik_max_ = num_ik_max;
}

void KinematicBase::Forward(const VectorXd& q, const Affine3d& base_trans, Affine3d& tip_trans) {
    tip_trans = base_trans;
    for (size_t i = 0; i < q.size(); i++) {
        Affine3d trans = CvtModelToTrans(model_.xyz[i], model_.axis[i], model_.type[i], q(i));
        tip_trans = tip_trans * trans;
    }
    Affine3d end_trans = CvtModelToTrans(model_.xyz.back(), model_.axis.back(), model_.type.back(), 0);
    tip_trans = tip_trans * end_trans;
}

bool KinematicBase::Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const VectorXd& init_q, VectorXd& q) {
    Affine3d tip_trans_d;
    VectorXd q_d = init_q;
    Forward(q_d, base_trans, tip_trans_d);
    
    // prepare
    Vector6d twist = Differentiate(tip_trans, tip_trans_d, 1.0);
    MatrixXd j = GetJacobian(q_d, base_trans);
    int num_ik = 0;
    while (true) {
        //Jacobian j_inv = j.Inverse();
        MatrixXd j_inv = SingularityLowSensitiveInverse(j);
        VectorXd dq = j_inv * twist;
        q_d += dq;
        
        Forward(q_d, base_trans, tip_trans_d);
        twist = Differentiate(tip_trans, tip_trans_d, 1.0);
        if (twist.norm() < 1e-6) break;
        
        j = GetJacobian(q_d, base_trans);
        num_ik++;
        if (num_ik > num_ik_max_) return false;
    }
    q = q_d;
    return true;
 }

Affine3d KinematicBase::CvtModelToTrans(const std::array<double, 3>& xyz, const std::array<double, 3>& axis, JointType type, double q) {
    Affine3d trans;
    if (type == ROTATE) {
        trans.translation() = Vector3d(xyz.data());
        trans.linear() = MatrixFromAxisAndValue(Vector3d(axis.data()), q);
    } else {
        trans.translation() << xyz[X], xyz[Y], xyz[Z];
    }
    return trans;
}

MatrixXd KinematicBase::GetJacobian(const VectorXd& q,const Affine3d& base_trans) {
    MatrixXd jacobian = MatrixXd::Zero(6, q.size());
    const double delta_q = 1e-5;
    for (int i = 0; i < q.size(); i++) {
        VectorXd q_p = q;
        q_p(i) += delta_q;
        Affine3d tip_trans_p;
        Forward(q_p, base_trans, tip_trans_p);
        VectorXd q_m = q;
        q_m(i) -= delta_q;
        Affine3d tip_trans_m;
        Forward(q_m, base_trans, tip_trans_m);
        jacobian.col(i) = Differentiate(tip_trans_p, tip_trans_m, 2 * delta_q);
    }
    return jacobian;
}

MatrixXd KinematicBase::SingularityLowSensitiveInverse(const MatrixXd& jacobian) {
    MatrixXd jacobian_plus;
    const MatrixXd& jacobian_t = jacobian.transpose();
    jacobian_plus = jacobian_t * (jacobian * jacobian_t + 0.01 * MatrixXd::Identity(jacobian.rows(), jacobian.rows())).inverse();
    return jacobian_plus;
}