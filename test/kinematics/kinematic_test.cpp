#include <gtest/gtest.h>
#include "kinematic_base.hpp"

constexpr double TOLERANCE = 0.1;

class KinematicTest : public ::testing::Test {
    protected:
    KinematicTest() : 
    kinematic_(3),
    model_(3) {
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
    }
    KinematicBase kinematic_;
    KinematicModel model_;
};

/**
 * @test test forward kinematic
 */
TEST_F(KinematicTest, ForwardKinematic) {
    VectorXd q(3);
    q << 45.0, 0.0, 45.0;
    q *= DEG_TO_RAD;
    Affine3d tip_trans = Affine3d::Identity();
    //kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
    std::cout << q.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans.translation().transpose() << std::endl;
}

/**
 * @test check inverse kinematic
 */
TEST_F(KinematicTest, CheckInverseKinematic) {
    // prepare q
    VectorXd q_expect(3);
    q_expect << 45.0, 45.0, 45.0;
    q_expect *= DEG_TO_RAD;
    // set tip_trans
    Affine3d tip_trans_expect = Affine3d::Identity();
    kinematic_.Forward(q_expect, Affine3d::Identity(), tip_trans_expect);
    std::cout << q_expect.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans_expect.translation().transpose() << std::endl;
    // check IK
    VectorXd q_standard(3);
    q_standard << 45.0, 30.0, 60.0;
    q_standard *= DEG_TO_RAD;
    VectorXd q_ik = VectorXd::Zero(3);
    bool ik_ret = kinematic_.Inverse(tip_trans_expect, Affine3d::Identity(), q_standard, q_ik);
    
    std::cout << q_ik.transpose() * RAD_TO_DEG << std::endl;
    for (int i = 0; i < q_expect.size(); i++) {
        EXPECT_NEAR(q_expect(i), q_ik(i), TOLERANCE);
    }
}