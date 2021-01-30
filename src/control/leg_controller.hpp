#ifndef LEG_H
#define LEG_H

#include "algorithm/kinematic.hpp"
#include "algorithm/math_const.hpp"
#include "State.hpp"
#include "SerialServo.hpp"

struct ActConfing {
    std::array<uint8_t, NUM_JOINT>  id;
    std::array<double, NUM_JOINT> gain;
    std::array<double, NUM_JOINT>  offset;
};

struct JointConfig {
    std::array<double, NUM_JOINT> q_min;
    std::array<double, NUM_JOINT>  q_max;
};

struct LegConfig {
    KinematicModel model;
    JointConfig joint;
    ActConfing act;
};

class Leg {
    public:
    Leg();
    void Config(LegConfig& config, const Joint& init_q, SerialServo& servo);
    void ForwardKinematic(const Joint& q, const Affine3d& base_trans, Affine3d& trans);
    bool InverseKinematic(const Affine3d& trans, const Affine3d& base_trans, bool& joint_limit, Joint& q);
    void SetJoint(const Joint& q);
    void GetJoint(Joint& q);
    void PassiveJoint(Joint& q);

    private:
    LegConfig config_;
    Kinematic kinematic_;
    SerialServo servo_;
    Joint q_pre_;    
    void ConvertToAct(const Joint& q, Joint& act_q);
    void ConvertToJoint(const Joint& act_q, Joint& q);
    bool LimitJoint(Joint& q);
};

#endif