#ifndef PUPPER_KINEMATIC_H
#define PUPPER_KINEMATIC_H

#include "kinematic_base.hpp"

enum Joint {
    PUPPER_ROLL,
    PUPPER_PITCH1,
    PUPPER_PITCH2,
    NUM_PUPPER_JOINT
};

class PupperKinematic : public KinematicBase {
    public:
    PupperKinematic();
    void Forward(const VectorXd& q, const Affine3d& base_trans, Affine3d& tip_trans) override;
    bool Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const VectorXd& init_q, VectorXd& q) override;
    static KinematicModel CreatePupperModel(const std::vector<std::array<double, XYZ>> xyz);
    private:
    VectorXd ConvertToCoupleJoint(const VectorXd& q);
    VectorXd ConvertFromCoupleJoint(const VectorXd& q_couple);
};

#endif