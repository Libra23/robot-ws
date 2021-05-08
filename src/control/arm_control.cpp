#include "arm_control.hpp"
#include "esp_log.h"

Arm::Arm() {
    if(IsQuadPupper()) {
        kinematic_.reset(new PupperKinematic());
    } else {
        kinematic_.reset(new KinematicBase());
    }
}

void Arm::Config(const ArmConfig& config, int arm_id) {
    kinematic_->Config(config.model, 30);
    q_min_ = VectorXd::Map(config.joint.q_min.data(), config.joint.q_min.size()) * DEG_TO_RAD;
    q_max_ = VectorXd::Map(config.joint.q_max.data(), config.joint.q_max.size()) * DEG_TO_RAD;
    q_pre_ = kinematic_->GetDefaultJoint(arm_id);
    act_gain_ = MatrixXd::Map(config.act.gain.data(), sqrt(config.act.gain.size()), sqrt(config.act.gain.size())).transpose();
    act_offset_ = VectorXd::Map(config.act.offset.data(), config.act.offset.size());
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

void Arm::GetDefault(int arm_id, VectorXd& q, const Affine3d& base_trans, Affine3d& trans) {
    q = kinematic_->GetDefaultJoint(arm_id);
    ForwardKinematic(q, base_trans, trans);
}

void Arm::ConvertToAct(const VectorXd& q, VectorXd& act_q) {
    act_q = act_gain_ * q + act_offset_;
}

void Arm::ConvertToJoint(const VectorXd& act_q, VectorXd& q) {
    q = act_gain_.inverse() * (act_q - act_offset_);
}

bool Arm::LimitJoint(VectorXd& q) {
    bool joint_limit = false;
    for (int i = 0; i < q.size(); i++) {
        if (q[i] < q_min_[i]) {
            joint_limit = true;
            q[i] = q_min_[i];
        } else if (q_max_[i] < q[i]) {
            joint_limit = true;
            q[i] = q_max_[i];
        }
    }
    return joint_limit;
}