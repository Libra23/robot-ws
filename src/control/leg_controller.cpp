#include "leg_controller.hpp"

Leg::Leg(){
}

void Leg::Config(LegConfig& config, const Joint& init_q) {
    config_ = config;
    for (int i = 0; i < NUM_JOINT; i++) {
        config_.joint.q_min[i] *= DEG_TO_RAD;
        config_.joint.q_max[i] *= DEG_TO_RAD;
    }
    kinematic_ = Kinematic();
    kinematic_.Config(config_.model, 30);
    q_pre_ = init_q;
}

void Leg::ForwardKinematic(const Joint& q, const Affine3d& base_trans, Affine3d& trans) {
    kinematic_.Forward(q, base_trans, trans);
}

bool Leg::InverseKinematic(const Affine3d& trans, const Affine3d& base_trans, bool& joint_limit, Joint& q) {
    bool ik_ret = kinematic_.Inverse(trans, base_trans, q_pre_, q);
    if(!ik_ret) q = q_pre_;
    joint_limit = LimitJoint(q);
    q_pre_ = q;
    return ik_ret;
}

void Leg::SetJoint(const Joint& q) {
    Joint act_q;
    ConvertToAct(q, act_q);
    for (int i = 0; i < NUM_JOINT; i++) {
        servo_.SetPosition(config_.act.id[i], act_q(i));
    }
}

void Leg::GetJoint(Joint& q) {
    Joint act_q;
    for (int i = 0; i < NUM_JOINT; i++) {
        act_q(i) = servo_.GetPosition(config_.act.id[i]);
    }
    ConvertToJoint(act_q, q);
}

void Leg::PassiveJoint(Joint& q) {
    Joint act_q;
    for (int i = 0; i < NUM_JOINT; i++) {
        act_q(i) = servo_.Passive(config_.act.id[i]);
    }
    ConvertToJoint(act_q, q);
}

void Leg::ConvertToAct(const Joint& q, Joint& act_q) {
    for (int i = 0; i < NUM_JOINT; i++) {
        act_q(i) = config_.act.gain[i] * q(i) + config_.act.offset[i];
    }
}

void Leg::ConvertToJoint(const Joint& act_q, Joint& q) {
    for (int i = 0; i < NUM_JOINT; i++) {
        q(i) = (act_q(i) - config_.act.offset[i]) / config_.act.gain[i];
    }
}

bool Leg::LimitJoint(Joint& q) {
    bool joint_limit = false;
    for (int i = 0; i < NUM_JOINT; i++) {
        if (q(i) < config_.joint.q_min[i]) {
            joint_limit = true;
            q(i) = config_.joint.q_min[i];
        } else if (config_.joint.q_max[i] < q(i)) {
            joint_limit = true;
            q(i) = config_.joint.q_max[i];
        }
    }
    return joint_limit;
}