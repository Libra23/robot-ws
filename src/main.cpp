#include <iostream>
#include "kinematics/kinematic_base.hpp"
#include "kinematics/pupper_kinematic.hpp"

int main () {
    std::cout << "Hello!" << std::endl;
    std::unique_ptr<KinematicBase> kinematic_;

    kinematic_.reset(new PupperKinematic());
    std::vector<std::array<double, XYZ>> xyz =  {{
                                                {{54.75, -36.25, 0.0}},  // Roll
                                                {{0.0, -39.0, 0.0}},     // Pitch1
                                                {{0.0, 0.0, -60.0}},    // Pitch2
                                                {{0.0, 0.0, -60.0}},    // Tip
                                                }};
    kinematic_->Config(PupperKinematic::CreatePupperModel(xyz), 50);

    // prepare q
    VectorXd q_expect(NUM_PUPPER_JOINT);
    q_expect << 0.0, 60.0, -60.0;
    q_expect *= DEG_TO_RAD;
    // set tip_trans
    Affine3d tip_trans_expect = Affine3d::Identity();
    kinematic_->Forward(q_expect, Affine3d::Identity(), tip_trans_expect);
    std::cout << "q_expect = " << q_expect.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans_expect.translation().transpose() << std::endl;

    // check IK
    VectorXd q_standard(NUM_PUPPER_JOINT);
    q_standard << 0.0, 0.0, 0.0;
    q_standard *= DEG_TO_RAD;
    VectorXd q_ik = VectorXd::Zero(NUM_PUPPER_JOINT);
    bool ik_ret = kinematic_->Inverse(tip_trans_expect, Affine3d::Identity(), q_standard, q_ik);
    std::cout << "q_ik = " << q_ik.transpose() * RAD_TO_DEG << std::endl;

    tip_trans_expect.translation()[Z] -= 0;
    std::cout << tip_trans_expect.translation().transpose() << std::endl;
    kinematic_->Inverse(tip_trans_expect, Affine3d::Identity(), q_standard, q_ik);
    std::cout << q_ik.transpose() * RAD_TO_DEG << std::endl;
    kinematic_->Forward(q_ik, Affine3d::Identity(), tip_trans_expect);
    std::cout << tip_trans_expect.translation().transpose() << std::endl;

    return 0;
}