#include <gtest/gtest.h>
#include "kinematic_base.hpp"
#include "pupper_kinematic.hpp"
#include <memory>

#define PLOT
#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

constexpr double TOLERANCE = 0.1;

class PupperKinematicTest : public ::testing::Test {
    protected:
    PupperKinematicTest();
    VectorXd Dynamics(const VectorXd& q, const VectorXd& qd, const VectorXd& qdd);
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

VectorXd PupperKinematicTest::Dynamics(const VectorXd& q, const VectorXd& qd, const VectorXd& qdd) {
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
    std::cout << q.transpose() * RAD_TO_DEG << std::endl;
    std::cout << tip_trans.translation().transpose() << std::endl;
    std::cout << kinematic_->GetDefaultJoint(2).transpose() * RAD_TO_DEG << std::endl;
}

/**
 * @test check inverse kinematic
 */
TEST_F(PupperKinematicTest, CheckInverseKinematic) {
    // prepare q
    std::vector<std::array<double, NUM_PUPPER_JOINT>> q_deg_list = {{
        {0, 1, -1},
        {30, 1, -1},
        {-30, 1, -1},
        {0, 30, 0},
        {0, -30, 0},
        {0, 30, -30},
        {0, 30, -60},
    }};

    for (size_t i = 0; i < q_deg_list.size(); i++) {
        std::cout << "\n ---test_num = " << i << std::endl;
        Affine3d tip_trans = Affine3d::Identity();
        VectorXd q_expect = VectorXd::Map(q_deg_list[i].data(), q_deg_list[i].size()) * DEG_TO_RAD;
        // Forward
        kinematic_->Forward(q_expect, Affine3d::Identity(), tip_trans);
        std::cout << "q_expect = " << q_expect.transpose() * RAD_TO_DEG << std::endl;
        std::cout <<  "tip_trans(XYZ) = "<< tip_trans.translation().transpose() << std::endl;
        // Inverse
        VectorXd q_default = kinematic_->GetDefaultJoint(0);
        VectorXd q_ik = VectorXd::Zero(NUM_PUPPER_JOINT);
        bool ik_ret = kinematic_->Inverse(tip_trans, Affine3d::Identity(), q_default, q_ik);
        EXPECT_TRUE(ik_ret);
        std::cout << " q_ik = " << q_ik.transpose() * RAD_TO_DEG << std::endl;
        // Check
        for (int i = 0; i < q_expect.size(); i++) {
            EXPECT_NEAR(q_expect(i), q_ik(i), TOLERANCE);
        }
    }
}

/**
 * @test check dynamics
 */
TEST_F(PupperKinematicTest, CheckDynamics) {
    const double mass = 0.5;
    const double target_height = 15.0;
    const double target_velocity = sqrt(2 * GRAVITY_CONSTANT * target_height);
    const double acc = 2.0;
    const double interval = target_velocity / acc;
    const double sample_time = 1e-2;
    const double resolution = interval / sample_time;
    std::cout << "interval[s] = " << interval << "resolution = " << resolution << std::endl;
    #ifdef PLOT
    std::vector<double> x(resolution), y0(resolution), y1(resolution), y2(resolution);
    #endif

    VectorXd q(NUM_PUPPER_JOINT);
    q << 0.0, 75.0, -150.0;
    q *= DEG_TO_RAD;
    Affine3d base_trans = Affine3d::Identity();
    Affine3d tip_trans;
    kinematic_->Forward(q, base_trans, tip_trans);
    std::cout << "tip_trans(XYZ) start = " << tip_trans.translation().transpose() << std::endl;
    VectorXd q_pre = q;
    VectorXd qd = VectorXd::Zero(NUM_PUPPER_JOINT);
    VectorXd qd_pre = qd;
    VectorXd qdd = VectorXd::Zero(NUM_PUPPER_JOINT);

    double t = 0;
    for (int i = 0; i < resolution; i++) {
        // update base ref
        base_trans.translation()[Z] = 0.5 * acc * t * t;
        // ik
        VectorXd q_ik = VectorXd::Zero(NUM_PUPPER_JOINT);
        bool ret = kinematic_->Inverse(tip_trans, base_trans, q, q_ik);
        const MatrixXd jacobian = kinematic_->GetJacobian(q_ik, base_trans);

        q = q_ik;
        qd = (q - q_pre) / sample_time;
        qdd = (qd - qd_pre) / sample_time;

        // effort
        Wrench ex_force;
        ex_force << 0, 0, mass * (GRAVITY_CONSTANT + acc) , 0.0, 0.0, 0.0;
        VectorXd effort = Dynamics(q_ik, qd, qdd) - jacobian.transpose() * ex_force;

        // update pre
        t += sample_time;
        q_pre = q;
        qd_pre = qd;

        Affine3d tip_trans_check;
        kinematic_->Forward(q, base_trans, tip_trans_check);

        #ifdef PLOT
        x[i] = t;
        constexpr bool SHOW_EFFORT= true;
        constexpr bool SHOW_QD = false;
        if (SHOW_EFFORT) {
            y0[i] = effort[0] * 0.001 / GRAVITY_CONSTANT * 100; // Nmm -> kg cm
            y1[i] = effort[1] * 0.001 / GRAVITY_CONSTANT * 100;
            y2[i] = effort[2] * 0.001 / GRAVITY_CONSTANT * 100;
        } else if (SHOW_QD) {
            y0[i] = qd[0] * RAD_TO_DEG; // rad/s -> deg/s
            y1[i] = qd[1] * RAD_TO_DEG;
            y2[i] = qd[2] * RAD_TO_DEG;
        } else {
            y0[i] = q[0] * RAD_TO_DEG; // rad -> deg.
            y1[i] = q[1] * RAD_TO_DEG;
            y2[i] = q[2] * RAD_TO_DEG;
        }

        #endif
    }

    kinematic_->Forward(q, base_trans, tip_trans);
    std::cout << "tip_trans(XYZ) end = " << tip_trans.translation().transpose() << std::endl;
    std::cout << "q = " << q.transpose() * RAD_TO_DEG << std::endl;
    #ifdef PLOT
    plt::named_plot("index0", x, y0);
    plt::legend(); 
    plt::named_plot("index1", x, y1);
    plt::legend(); 
    plt::named_plot("index2", x, y2);
    plt::legend(); 
    plt::show();
    #endif
}