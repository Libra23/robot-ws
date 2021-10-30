#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "constant/math_const.hpp"
#include "kinematics/kinematic_base.hpp"
#include "kinematics/pupper_kinematic.hpp"
#include "control_data/robot_data.hpp"
#include <memory>

struct JointConfig {
    std::vector<double> q_min;
    std::vector<double> q_max;
    JointConfig() : 
        q_min(NUM_JOINT, 0),
        q_max(NUM_JOINT, 0) {}
};

struct ActConfig {
    std::vector<int8_t> id;
    std::vector<double> gain;
    std::vector<double>  offset;
    ActConfig() : 
        id(NUM_JOINT, -1),
        gain(NUM_JOINT * NUM_JOINT, 0),
        offset(NUM_JOINT, 0) {}
};
struct ArmConfig {
    KinematicModel model;
    JointConfig joint;
    ActConfig act;
    ArmConfig() : 
        model(KinematicModel()),
        joint(JointConfig()),
        act(ActConfig()) {}
};

class Arm {
    public:
    Arm();
    void Config(const ArmConfig& config, int arm_id);
    void ForwardKinematic(const VectorXd& q, const Affine3d& base_trans, Affine3d& trans);
    bool InverseKinematic(const Affine3d& trans, const Affine3d& base_trans, bool& is_joint_limit, VectorXd& q);
    void GetDefault(int arm_id, VectorXd& q, const Affine3d& base_trans, Affine3d& trans);
    void ConvertToAct(const VectorXd& q, VectorXd& act_q);
    void ConvertToJoint(const VectorXd& act_q, VectorXd& q);

    private:
    std::unique_ptr<KinematicBase> kinematic_;
    VectorXd q_min_;
    VectorXd q_max_;
    VectorXd q_pre_;
    MatrixXd act_gain_;
    MatrixXd act_gain_inverse_;
    VectorXd act_offset_;
    bool LimitJoint(VectorXd& q);
};

#endif