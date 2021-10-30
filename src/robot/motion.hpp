#ifndef MOTION_H
#define MOTION_H

#include "control_data/robot_data.hpp"
#include "constant/math_const.hpp"

struct MotionParam {
    double stride;
    double direction;
};

enum Mode {
    STOP,
    PASSIVE,
    ACTIVE
};

class Motion{
    public:
    Motion();
    void Config(const std::array<double, XYZ>& cog);
    void Start(double time, double stride, double direction, double slide, double height, double speed, const RobotState& state);
    void UpdateMotion(double time, const RobotRef& ref, const RobotState& state, RobotOut& out);

    private:
    Vector3d cog_;
    Vector3d rpy_error_integral_;
    double time_;
    double stride_, direction_, slide_, height_, frequency_;
    Mode previous_mode_;
    std::array<Vector3d, NUM_ARM> pos_start_;
    void Walk(double time, RobotOut& out);
    void StraightUnit(double time, double duty, Vector3d& pos);
    void SlideSpin(double time, const RobotRef& ref, const RobotState& state, RobotOut& out);
    void Slide(double time, const RobotRef& ref, const RobotState& state, RobotOut& out);
};

#endif