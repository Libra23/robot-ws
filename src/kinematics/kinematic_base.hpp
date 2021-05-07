#ifndef KINEMATIC_BASE_H
#define KINEMATIC_BASE_H

#include "algorithm/math_const.hpp"
#include "algorithm/math_utility.hpp"
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
    KinematicModel() {};
};

class KinematicBase {
    public:
    KinematicBase();
    virtual ~KinematicBase();
    void Config(const KinematicModel& model, uint8_t num_ik_max = 20);
    virtual void Forward(const VectorXd& q, const Affine3d& base_trans, Affine3d& tip_trans);
    virtual bool Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const VectorXd& init_q, VectorXd& q);
    MatrixXd GetJacobian(const VectorXd& q, const Affine3d& base_trans);

    protected:
    KinematicModel model_;
    Affine3d CvtModelToTrans(const std::array<double, 3>& xyz, const std::array<double, 3>& axis, JointType type, double q);

    private:
    uint8_t num_ik_max_;
    MatrixXd SingularityLowSensitiveInverse(const MatrixXd& jacobian);
};

#endif