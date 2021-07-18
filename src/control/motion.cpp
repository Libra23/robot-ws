#include "motion.hpp"
#include "algorithm/math_const.hpp"

Motion::Motion() {

}

void Motion::Config(const std::array<double, XYZ>& cog) {
    cog_ = Vector3d(cog.data());
    rpy_error_integral_ = Vector3d::Zero();
    previous_mode_ = Mode::STOP;
}

void Motion::Start(double time, double stride, double direction, double slide, double height, double speed, const RobotState& state) {
    time_ = time;
    stride_ = stride;
    direction_ = direction * DEG_TO_RAD;
    slide_ = slide;
    height_ = height;
    frequency_ = speed / stride;
    // set start pos
    for (size_t i = 0; i < NUM_ARM; i++) {
        pos_start_[i] = state.arm[i].trans.translation();
    }
}

void Motion::UpdateMotion(double time, const RobotRef& ref, const RobotState& state, RobotOut& out) {
    // initialize
    out.body = ref.body;
    out.arm = ref.arm;
    Mode mode = Mode::STOP;

    // check mode
    const bool is_passive = false;
    if (is_passive) {
        mode = Mode::PASSIVE;
        out.arm = state.arm;
    } else {
        mode = Mode::ACTIVE;
        //SlideSpin(time, ref, state, out);
        Walk(time, out);
    }
    previous_mode_ = mode;
}

void Motion::Walk(double time, RobotOut& out) {
    std::array<double, NUM_ARM> phase;
    phase[LEFT_FRONT] = 0.0;
    phase[RIGHT_BACK] = 270.0;
    phase[RIGHT_FRONT] = 180.0;
    phase[LEFT_BACK] = 90.0;

    for (size_t i = 0; i < NUM_ARM; i++) {
        Vector3d unit;
        Vector3d pos;
        StraightUnit(2 * PI * frequency_ * time + phase[i] * DEG_TO_RAD, 0.8, unit);
        pos(X) = stride_ * unit(X) * cos(direction_) - slide_ * unit(Y) * sin(direction_) + pos_start_[i](X);
        pos(Y) = stride_ * unit(X) * sin(direction_) + slide_ * unit(Y) * cos(direction_) + pos_start_[i](Y);
        pos(Z) = height_ * unit(Z) + pos_start_[i](Z);
        out.arm[i].trans.translation() = pos;
    }
}

void Motion::StraightUnit(double time, double duty, Vector3d& pos) {
    double t = fmod(time, 2 * PI);
    pos(Y) = 0.0;
    if (0 <= t && t < 2 * PI * (1 - duty)) {
        pos(X) = 0.5 * (1.0 - cos(0.5 * t / (1 - duty)));
        pos(Z) = 0.5 * (1.0 - cos(t / (1 - duty)));
    } else {
        pos(X) = 0.5 * (1.0 - cos(0.5 * t / duty - PI / duty));
        pos(Z) = 0.0;
    }
}

void Motion::SlideSpin(double time, const RobotRef& ref, const RobotState& state, RobotOut& out) {
    // slide body
    out.body.trans.translation() = ref.body.trans.translation();

    // spin body
    /*
    const double kpi = 0.05;
    const Vector3d rpy_error = Rpy(ref.body.trans.rotation()) - Rpy(state.body.trans.rotation());
    rpy_error_integral_ += (rpy_error * kpi);
    out.body.trans.linear() = MatrixFromRpy(rpy_error_integral_);
    */
    out.body.trans.linear() = ref.body.trans.linear();

}