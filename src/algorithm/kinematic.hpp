#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "math_const.hpp"
#include "math_utility.hpp"
#include <vector>
#include <array>

enum JointType {
    ROTATE,
    SLIDE,
    FIXED
};

struct KinematicModel {
    std::vector<std::array<double, XYZ>> xyz;
    std::vector<std::array<double, XYZ>> axis;
    std::vector<JointType> type;
    KinematicModel(uint8_t num_joint) : 
        xyz(num_joint + 1),
        axis(num_joint + 1),
        type(num_joint + 1) {}
};

class Kinematic {
    public:
    Kinematic(uint8_t num_joint);
    void Config(const KinematicModel& model, uint8_t num_ik_max = 20);
    void Forward(const VectorXd& q, const Affine3d& base_trans, Affine3d& tip_trans);
    bool Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const VectorXd& init_q, VectorXd& q);

    private:
    KinematicModel model_;
    uint8_t num_ik_max_;
    Affine3d CvtModelToTrans(const std::array<double, 3>& xyz, const std::array<double, 3>& axis, JointType type, double q);
    MatrixXd GetJacobian(const VectorXd& q, const Affine3d& base_trans, const Affine3d& tip_trans);
    MatrixXd SingularityLowSensitiveInverse(const MatrixXd& jacobian);
};
#endif