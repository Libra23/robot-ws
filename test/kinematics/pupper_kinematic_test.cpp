#include <gtest/gtest.h>
#include "kinematic_base.hpp"
#include "pupper_kinematic.hpp"
#include <memory>

constexpr double TOLERANCE = 0.1;

class PupperKinematicTest : public ::testing::Test {
    protected:
    PupperKinematicTest();
    VectorXd Dynamics(const VectorXd q, const VectorXd qd, const VectorXd qdd);
    std::unique_ptr<KinematicBase> kinematic_;
};

PupperKinematicTest::PupperKinematicTest() {
    kinematic_.reset(new PupperKinematic());
    KinematicModel model;
    model.xyz = {{
                {{54.75, -36.25, 0.0}},  // Roll
                {{0.0, -39.0, 0.0}},     // Pitch1
                {{0.0, 0.0, -60.0}},    // Pitch2
                {{0.0, 0.0, -60.0}},    // Tip
                }};
    model.axis = {{
                 {{1.0, 0.0, 0.0}},      // Roll
                 {{0.0, 1.0, 0.0}},      // Pitch1
                 {{0.0, 1.0, 0.0}},      // Pitch2
                 {{1.0, 0.0, 0.0}},      // Tip
                 }};
    model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    kinematic_->Config(model, 50);
}

VectorXd PupperKinematicTest::Dynamics(const VectorXd q, const VectorXd qd, const VectorXd qdd) {
    const double m1 = 0.05;
    const double m2 = 0.05;
    const double l1 = 60.0;
    const double l2 = 60.0;
    const double lg1 = 30.0;
    const double lg2 = 30.0;
    const double i1 = m1 * l1 * l1 / 12;
    const double i2 = m2 * l2 * l2 / 12;
    const double q1 = q[1];
    const double q2 = q[2];
    const double qd1 = qd[1];
    const double qd2 = qd[2];

    MatrixXd M(2, 2);
    M << m1 * lg1 * lg1 + m2 * (l1 * l1 + 2 * l1 * lg2 * cos(q2) + lg2 * lg2) + i1 + i2, m2 * (l1 * lg2 * cos(q2) + lg2 * lg2) + i2,
         m2 * (l1 * lg2 * cos(q2) + lg2 * lg2) + i2, m2 * lg2 * lg2 + i2;
    VectorXd h(2);
    h << -m2 * l1 * lg2 * (2 * qd1 + qd2) * sin(q2) * qd2, m2 * l1 * lg2 * qd1 * qd1 * sin(q2);
    VectorXd g(2);
    g << m1 * GRAVITY_CONSTANT * lg1 * sin(q1) + m2 * GRAVITY_CONSTANT * (l1 * sin(q1) + lg2 * sin(q1 + q2)), m2 * GRAVITY_CONSTANT * lg2 * sin(q1 + q2);
    
    VectorXd effort = VectorXd::Zero(3);
    effort.tail(2) = M * qdd.tail(2) + h + g;

    return effort;
}

/**
 * @test test forward kinematic
 */
TEST_F(PupperKinematicTest, ForwardKinematic) {
    VectorXd q(NUM_PUPPER_JOINT);
    q << 0.0, 30.0, -30.0;
    q *= DEG_TO_RAD;
    Affine3d tip_trans = Affine3d::Identity();
    kinematic_->Forward(q, Affine3d::Identity(), tip_trans);
    //std::cout << q.transpose() * RAD_TO_DEG << std::endl;
    //std::cout << tip_trans.translation().transpose() << std::endl;
    std::cout << kinematic_->GetDefaultJoint(2) * RAD_TO_DEG << std::endl;
    EXPECT_FALSE(true);
}

/**
 * @test check inverse kinematic
 */
TEST_F(PupperKinematicTest, CheckInverseKinematic) {
    // prepare q
    VectorXd q_expect(NUM_PUPPER_JOINT);
    q_expect << 0.0, 30.0, -30.0;
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

/**
 * @test check dynamics
 */
TEST_F(PupperKinematicTest, CheckDynamics) {
    const double mass = 0.5;

    VectorXd q(NUM_PUPPER_JOINT);
    q << 0.0, 60.0, -120.0;
    q *= DEG_TO_RAD;
    Affine3d tip_trans;
    kinematic_->Forward(q, Affine3d::Identity(), tip_trans);

    const int num_test = 2;
    const double sample_time = 1e-3;
    VectorXd q_pre = q;
    VectorXd qd = VectorXd::Zero(NUM_PUPPER_JOINT);
    VectorXd qd_pre = qd;
    VectorXd qdd = VectorXd::Zero(NUM_PUPPER_JOINT);
    for (int i = 0; i < num_test; i++) {
        // update base ref

        // ik
        VectorXd q_ik;
        kinematic_->Inverse(tip_trans, Affine3d::Identity(), q, q_ik);
        const MatrixXd jacobian = kinematic_->GetJacobian(q_ik, Affine3d::Identity());

        qd = (q_ik - q_pre) / sample_time;
        qdd = (qd - qd_pre) / sample_time;

        // effort
        VectorXd ex_force(XYZ);
        ex_force << 0, 0, mass * GRAVITY_CONSTANT;
        VectorXd effort = Dynamics(q_ik, qd, qdd) + jacobian.transpose() * ex_force;

        std::cout << "effort = " << effort.transpose() << std::endl;

        // update pre
        q_pre = q_ik;
        qd_pre = qd;
    } 
}