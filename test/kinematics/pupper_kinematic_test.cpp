#include <gtest/gtest.h>
#include "kinematic_base.hpp"
#include "pupper_kinematic.hpp"
#include <memory>

constexpr double TOLERANCE = 0.1;

class PupperKinematicTest : public ::testing::Test {
    protected:
    PupperKinematicTest();
    std::unique_ptr<KinematicBase> kinematic_;
};

PupperKinematicTest::PupperKinematicTest() {
    kinematic_.reset(new PupperKinematic());
    std::vector<std::array<double, XYZ>> xyz =  {{
                                                {{54.75, -36.25, 0.0}},  // Roll
                                                {{0.0, -39.0, 0.0}},     // Pitch1
                                                {{0.0, 0.0, -60.0}},    // Pitch2
                                                {{0.0, 0.0, -60.0}},    // Tip
                                                }};
    kinematic_->Config(PupperKinematic::CreatePupperModel(xyz), 50);
}

/**
 * @test test forward kinematic
 */
TEST_F(PupperKinematicTest, ForwardKinematic) {
    VectorXd q(NUM_PUPPER_JOINT);
    q << 0.0, 45.0, -45.0;
    q *= DEG_TO_RAD;
    Affine3d tip_trans = Affine3d::Identity();
    kinematic_->Forward(q, Affine3d::Identity(), tip_trans);
    std::cout << q.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans.translation().transpose() << std::endl;
    EXPECT_FALSE(true);
}

/**
 * @test check inverse kinematic
 */
TEST_F(PupperKinematicTest, CheckInverseKinematic) {
    // prepare q
    VectorXd q_expect(NUM_PUPPER_JOINT);
    q_expect << 0.0, 45.0, -45.0;
    q_expect *= DEG_TO_RAD;
    // set tip_trans
    Affine3d tip_trans_expect = Affine3d::Identity();
    kinematic_->Forward(q_expect, Affine3d::Identity(), tip_trans_expect);
    std::cout << q_expect.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans_expect.translation().transpose() << std::endl;
    // check IK
    VectorXd q_standard(NUM_PUPPER_JOINT);
    q_standard << 0.0, 0.0, 0.0;
    q_standard *= DEG_TO_RAD;
    VectorXd q_ik = VectorXd::Zero(NUM_PUPPER_JOINT);
    bool ik_ret = kinematic_->Inverse(tip_trans_expect, Affine3d::Identity(), q_standard, q_ik);
    
    std::cout << q_ik.transpose() * RAD_TO_DEG << std::endl;
    for (int i = 0; i < q_expect.size(); i++) {
        EXPECT_NEAR(q_expect(i), q_ik(i), TOLERANCE);
    }
}