#include "robot.hpp"

Robot::Robot(){}

void Robot::Config(SerialServo& servo) {
    SetHardwareConfig(config_);
    for (size_t i = 0; i < leg_.size(); i++) {
        leg_[i].Config(config_.leg_config[i], GetDefaultJoint(static_cast<LegID>(i)), servo);
    }
    sensor_.Config();
}

void Robot::UpdateState(RobotState& state) {    
    // sensor
    sensor_.UpdateSensorState(state.sensor_state);
    // body
    const Vector3d rpy = Vector3d(atan2(-state.sensor_state.accel[Y], -state.sensor_state.accel[Z]),
                                  atan2(state.sensor_state.accel[X], -state.sensor_state.accel[Z]),
                                  0.0);
    state.body_state.trans.translation() = Vector3d(0.0, 0.0, 0.0);
    state.body_state.trans.linear() = MatrixFromRpy(rpy);
    // leg
    for (size_t i = 0; i < leg_.size(); i++) {
        leg_[i].GetJoint(state.leg_state[i].q);
        leg_[i].ForwardKinematic(state.leg_state[i].q, state.body_state.trans, state.leg_state[i].trans);
    }
}

void Robot::Control(RobotOut& out) {
    for (size_t i = 0; i < leg_.size(); i++) {
        if (out.is_passive) {
            leg_[i].PassiveJoint(out.leg_state[i].q);
        } else {
            bool joint_limit = false;
            bool ik_ret=leg_[i].InverseKinematic(out.leg_state[i].trans, out.body_state.trans, joint_limit, out.leg_state[i].q);
            if (!ik_ret) {
                Serial.print(" ID = ");Serial.print(i);Serial.println("fail ik");
            } else if (joint_limit) {
                Serial.print(" ID = ");Serial.print(i);Serial.println("joint limit");
            }
            leg_[i].SetJoint(out.leg_state[i].q);
        }
    }
}

void Robot::SetHardwareConfig(RobotConfig& config) {
    config.cog = {{0.0, 0.0, 0.0}};

    LegConfig leg_config;
    // LEFT FRONT
    leg_config.model.xyz =  {{
                            {{27.5, 47.63, 0.0}},   // Yaw
                            {{33.25, 0.0, 0.0}},    // Pitch1
                            {{60.0, 0.0, 0.0}},     // Pitch2
                            {{120.0, 0.0, 0.0}},    // Tip
                            }};
    leg_config.model.axis = {{
                            {{0.0, 0.0, 1.0}},      // Yaw
                            {{0.0, 1.0, 0.0}},      // Pitch1
                            {{0.0, 1.0, 0.0}},      // Pitch2
                            {{0.0, 0.0, 0.0}},      // Tip
                            }};
    leg_config.model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    leg_config.joint.q_min = {{0.0, -60.0, 0.0}};
    leg_config.joint.q_max = {{90.0, 75.0, 150.0}};
    leg_config.act.id = {{0, 1, 2}};
    leg_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    leg_config.act.offset = {{60.0, 0.0, -90.0}};
    config.leg_config[LEFT_FRONT] = leg_config;

    // LEFT BACK
    leg_config.model.xyz =  {{
                            {{-27.5, 47.63, 0.0}},  // Yaw
                            {{33.25, 0.0, 0.0}},    // Pitch1
                            {{60.0, 0.0, 0.0}},     // Pitch2
                            {{120.0, 0.0, 0.0}},    // Tip
                            }};
    leg_config.model.axis = {{
                            {{0.0, 0.0, 1.0}},      // Yaw
                            {{0.0, 1.0, 0.0}},      // Pitch1
                            {{0.0, 1.0, 0.0}},      // Pitch2
                            {{0.0, 0.0, 0.0}},      // Tip
                            }};
    leg_config.model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    leg_config.joint.q_min = {{90.0, -60.0, 0.0}};
    leg_config.joint.q_max = {{180.0, 75.0, 150.0}};
    leg_config.act.id = {{3, 4, 5}};
    leg_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    leg_config.act.offset = {{120.0, 0.0, -90.0}};
    config.leg_config[LEFT_BACK] = leg_config;

    // RIGHT FRONT
    leg_config.model.xyz =  {{
                            {{27.5, -47.63, 0.0}},  // Yaw
                            {{33.25, 0.0, 0.0}},    // Pitch1
                            {{60.0, 0.0, 0.0}},     // Pitch2
                            {{120.0, 0.0, 0.0}},    // Tip
                            }};
    leg_config.model.axis = {{
                            {{0.0, 0.0, 1.0}},      // Yaw
                            {{0.0, 1.0, 0.0}},      // Pitch1
                            {{0.0, 1.0, 0.0}},      // Pitch2
                            {{0.0, 0.0, 0.0}},      // Tip
                            }};
    leg_config.model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    leg_config.joint.q_min = {{-90.0, -60.0, 0.0}};
    leg_config.joint.q_max = {{0.0, 75.0, 150.0}};
    leg_config.act.id = {{6, 7, 8}};
    leg_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    leg_config.act.offset = {{-60.0, 0.0, -90.0}};
    config.leg_config[RIGHT_FRONT] = leg_config;

    // RIGHT BACK
    leg_config.model.xyz =  {{
                            {{-27.5, -47.63, 0.0}}, // Yaw
                            {{33.25, 0.0, 0.0}},    // Pitch1
                            {{60.0, 0.0, 0.0}},     // Pitch2
                            {{120.0, 0.0, 0.0}},    // Tip
                            }};
    leg_config.model.axis = {{
                            {{0.0, 0.0, 1.0}},      // Yaw
                            {{0.0, 1.0, 0.0}},      // Pitch1
                            {{0.0, 1.0, 0.0}},      // Pitch2
                            {{0.0, 0.0, 0.0}},      // Tip
                            }};
    leg_config.model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    leg_config.joint.q_min = {{-180.0, -60.0, 0.0}};
    leg_config.joint.q_max = {{-90.0, 75.0, 150.0}};
    leg_config.act.id = {{9, 10, 11}};
    leg_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    leg_config.act.offset = {{-120.0, 0.0, -90.0}};
    config.leg_config[RIGHT_BACK] = leg_config;
}

RobotConfig& Robot::GetHardwareConfig() {
    return config_;
}

void Robot::GetDefaultRef(RobotRef& ref) {
    // body
    ref.body_state.trans = Affine3d::Identity();
    ref.body_state.trans.linear() = MatrixFromRpy(Vector3d(0.0, 0.0, 0.0));

    // leg
    for (size_t i = 0; i < ref.leg_state.size(); i++) {
        ref.leg_state[i].q = GetDefaultJoint(static_cast<LegID>(i));
        leg_[i].ForwardKinematic(ref.leg_state[i].q, Affine3d::Identity(), ref.leg_state[i].trans);
    }
}

Vector3d Robot::GetDefaultJoint(const LegID& id) {
    Vector3d q_deg = Vector3d(0.0, 0.0, 0.0);
    if (id == LEFT_FRONT) {
        q_deg = Vector3d(60.0, 45.0, 45.0);
    } else if (id == LEFT_BACK) {
        q_deg = Vector3d(150.0, 45.0, 45.0);
    } else if (id == RIGHT_FRONT) {
        q_deg = Vector3d(-60.0, 45.0, 45.0);
    } else if (id == RIGHT_BACK) {
        q_deg = Vector3d(-150.0, 45.0, 45.0);
    }
    return q_deg * DEG_TO_RAD;
}