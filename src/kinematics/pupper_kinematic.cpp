#include "pupper_kinematic.hpp"
#include <iostream>

PupperKinematic::PupperKinematic() : KinematicBase() {

}

bool PupperKinematic::Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const VectorXd& init_q, VectorXd& q) {
    Affine3d local_trans = tip_trans * base_trans.inverse();
    const double& l_roll_pitch1 = model_.xyz[1][Y];
    const double& l_pitch1_pitch2 = model_.xyz[2][Z];
    const double& l_pitch2_tip = model_.xyz[3][Z];
    const double& x = local_trans.translation()[X] - model_.xyz[0][X];
    const double& y = local_trans.translation()[Y] - model_.xyz[0][Y];
    const double& z = local_trans.translation()[Z] - model_.xyz[0][Z];

    const double lz = sqrt(y * y + z * z - l_roll_pitch1 * l_roll_pitch1);
    q[PUPPER_ROLL] = atan2(z, y) + atan2(lz, l_roll_pitch1);

    const double l_pitch1_tip = sqrt(x * x +  lz * lz);
    const double cos_gamma = (l_pitch1_pitch2 * l_pitch1_pitch2 + l_pitch2_tip * l_pitch2_tip -l_pitch1_tip * l_pitch1_tip) / 
                             (2 * l_pitch1_pitch2 * l_pitch2_tip);
    if (fabs(cos_gamma) > 1) return false;
    q[PUPPER_PITCH2] = acos(cos_gamma) - PI;

    const double sin_beta = -l_pitch2_tip / l_pitch1_tip * sin(acos(cos_gamma));
    if (fabs(sin_beta) > 1) return false;
    q[PUPPER_PITCH1] = atan2(x, lz) + asin(sin_beta);

    return true;
}

VectorXd PupperKinematic::GetDefaultJoint(int arm_id) {
    VectorXd q(NUM_PUPPER_JOINT);
    q << 0.0, 45.0, -90.0;
    return q * DEG_TO_RAD;
}

KinematicModel PupperKinematic::CreatePupperModel(const std::vector<std::array<double, XYZ>> xyz) {
    KinematicModel model;
    model.xyz = xyz;
    model.axis = {{
                 {{1.0, 0.0, 0.0}},      // Roll
                 {{0.0, 1.0, 0.0}},      // Pitch1
                 {{0.0, 1.0, 0.0}},      // Pitch2
                 {{1.0, 0.0, 0.0}},      // Tip
                 }};
    model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};

    return model;
}