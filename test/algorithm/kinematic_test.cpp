#include <gtest/gtest.h>
#include "kinematic.hpp"

constexpr double TOLERANCE = 0.1;

class KinematicTest : public ::testing::Test {
    protected:
    KinematicTest() {
        model_.xyz = {{
                {{27.5, 47.63, 0.0}},   // Yaw
                {{33.25, 0.0, 0.0}},    // Pitch1
                {{60.0, 0.0, 0.0}},     // Pitch2
                {{120.0, 0.0, 0.0}},    // Tip
                }};
        model_.axis = {{
                 {{0.0, 0.0, 1.0}},      // Yaw
                 {{0.0, 1.0, 0.0}},      // Pitch1
                 {{0.0, 1.0, 0.0}},      // Pitch2
                 {{0.0, 0.0, 0.0}},      // Tip
                 }};
        model_.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
        kinematic_.Config(model_, 50);
        kinematic_.SetConstant(1500.0, 0.01);   // set w & k
    }
    Kinematic kinematic_;
    KinematicModel model_;
};

/**
 * @test forward kinematic
 */
TEST_F(KinematicTest, ForwardKinematic) {
    if (NUM_JOINT != 3) return;
    Joint q;
    q << 45.0, 0.0, 45.0;
    q *= DEG_TO_RAD;
    Affine3d tip_trans;
    kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
    std::cout << q.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans.translation().transpose() << std::endl;
}

TEST_F(KinematicTest, CheckInverseKinematic) {
    // prepare q
    Joint q_expect;
    q_expect << 45.0, 45.0, 45.0;
    q_expect *= DEG_TO_RAD;
    // set tip_trans
    Affine3d tip_trans_expect;
    kinematic_.Forward(q_expect, Affine3d::Identity(), tip_trans_expect);
    std::cout << q_expect.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans_expect.translation().transpose() << std::endl;
    // check IK
    Joint q_standard;
    q_standard << 45.0, 30.0, 60.0;
    q_standard *= DEG_TO_RAD;
    Joint q_ik;
    bool ik_ret = kinematic_.Inverse(tip_trans_expect, Affine3d::Identity(), q_standard, q_ik);
    std::cout << q_ik.transpose() * RAD_TO_DEG << std::endl;
    for (int i = 0; i < NUM_JOINT; i++) {
        EXPECT_NEAR(q_expect(i), q_ik(i), TOLERANCE);
    }
}