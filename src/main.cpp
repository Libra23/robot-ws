#include <iostream>
#include <iomanip>
#include "utility/kinematic.hpp"

Kinematic kinematic_;

int main () {
    std::cout << "Hello1!" << std::endl;
    
    KinematicModel model;
    model.xyz = {{
                {{27.5, 47.63, 0.0}},   // Yaw
                {{33.25, 0.0, 0.0}},    // Pitch1
                {{60.0, 0.0, 0.0}},     // Pitch2
                {{120.0, 0.0, 0.0}},    // Tip
                }};
    model.axis = {{
                 {{0.0, 0.0, 1.0}},      // Yaw
                 {{0.0, 1.0, 0.0}},      // Pitch1
                 {{0.0, 1.0, 0.0}},      // Pitch2
                 {{0.0, 0.0, 0.0}},      // Tip
                 }};
    model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    kinematic_.Config(model);
    kinematic_.SetConstant(1500.0, 0.01);   // set w & k
    
    // check FK
    std::cout << " check FK " << std::endl;
    Joint q;
    q << 45.0, 0.0, 45.0;
    q *= DEG_TO_RAD;
    Affine3d tip_trans;
    kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
    std::cout << std::fixed << std::setprecision(3) << q.transpose() * RAD_TO_DEG << std::endl;
    std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
    
    // check Ik
    std::cout << " check IK " << std::endl;
    const Joint q_pre = q;
    const Affine3d tip_trans_pre = tip_trans;
    q = q_pre;
    tip_trans.translation() = tip_trans_pre.translation() + Vector3d(0.0, 0.0, 0.002);
    bool ik_ret = kinematic_.Inverse(tip_trans, Affine3d::Identity(), q_pre, q);
    std::cout << " result " << ik_ret << std::endl;
    std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
    std::cout << std::fixed << std::setprecision(5) << q.transpose() * RAD_TO_DEG << std::endl;
    kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
    std::cout << " compare IK & FK " << ik_ret << std::endl;
    std::cout << std::fixed << std::setprecision(3) << tip_trans.translation().transpose() << std::endl;
    std::cout << "Hello2!" << std::endl;
}