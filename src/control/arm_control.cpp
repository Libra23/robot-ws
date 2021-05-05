#include "arm_control.hpp"

Arm::Arm() : 
    kinematic_(new KinematicBase(NUM_JOINT)){}

void Arm::Config(const ArmConfig& config, const VectorXd& init_q) {
    config_ = config;
    for (int i = 0; i < init_q.size(); i++) {
        config_.joint.q_min[i] *= DEG_TO_RAD;
        config_.joint.q_max[i] *= DEG_TO_RAD;
    }

    if(IsQuadPupper()) {
        kinematic_.reset(new PupperKinematic());
    }
    kinematic_->Config(config_.model, 30);
    q_pre_ = init_q;
}

void Arm::ForwardKinematic(const VectorXd& q, const Affine3d& base_trans, Affine3d& trans) {
    kinematic_->Forward(q, base_trans, trans);
}

bool Arm::InverseKinematic(const Affine3d& trans, const Affine3d& base_trans, bool& is_joint_limit, VectorXd& q) {
    bool ik_ret = kinematic_->Inverse(trans, base_trans, q_pre_, q);
    if(!ik_ret) q = q_pre_;
    is_joint_limit = LimitJoint(q);
    q_pre_ = q;
    return ik_ret;
}

void Arm::ConvertToAct(const VectorXd& q, VectorXd& act_q) {
    for (int i = 0; i < q.size(); i++) {
        act_q[i] = config_.act.gain[i] * q[i] + config_.act.offset[i];
    }
}

void Arm::ConvertToJoint(const VectorXd& act_q, VectorXd& q) {
    for (int i = 0; i < q.size(); i++) {
        q[i] = (act_q[i] - config_.act.offset[i]) / config_.act.gain[i];
    }
}

bool Arm::LimitJoint(VectorXd& q) {
    bool joint_limit = false;
    for (int i = 0; i < q.size(); i++) {
        if (q[i] < config_.joint.q_min[i]) {
            joint_limit = true;
            q[i] = config_.joint.q_min[i];
        } else if (config_.joint.q_max[i] < q[i]) {
            joint_limit = true;
            q[i] = config_.joint.q_max[i];
        }
    }
    return joint_limit;
}