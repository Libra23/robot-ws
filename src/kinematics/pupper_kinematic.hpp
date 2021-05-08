#ifndef PUPPER_KINEMATIC_H
#define PUPPER_KINEMATIC_H

#include "kinematic_base.hpp"

enum PupperJointId {
    PUPPER_ROLL,
    PUPPER_PITCH1,
    PUPPER_PITCH2,
    NUM_PUPPER_JOINT
};

class PupperKinematic : public KinematicBase {
    public:
    PupperKinematic();
    bool Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const VectorXd& init_q, VectorXd& q) override;
    VectorXd GetDefaultJoint(int arm_id) override;
    static KinematicModel CreatePupperModel(const std::vector<std::array<double, XYZ>> xyz);
};

#endif