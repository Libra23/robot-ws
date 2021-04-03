#include "robot_control.hpp"
#include "esp_log.h"
/**
 * @brief Global parameter
 */
extern ShareMemory<OutputState> output_memory_; //!< defined at io_interface
extern ShareMemory<InputState> input_memory_;   //!< defined at io_interface

/**
 * @class Robot
 */
Robot::Robot() {
    ESP_LOGI("Robot", "Constructor");
}

void Robot::Thread() {
    ESP_LOGI("Robot", "Thread");
    // set config
    CreateConfig(config_);
    for (size_t i = 0; i < arm_.size(); i++) {
        arm_[i].Config(config_.arm_config[i], GetDefaultJoint(static_cast<ArmId>(i)));
    }

    while(true) {
        // input
        
        // set ref
        RobotRef ref;
        GetDefaultRef(ref);

        // output
        OutputState output;
        ConvertOutput(ref, output);
        output_memory_.Write(output);
        delay(10);
    }
}

void Robot::CreateConfig(RobotConfig& config) {
    config.cog = {{0.0, 0.0, 0.0}};
    // arm model
    ArmConfig arm_config;
    arm_config.model.xyz =  {{
                            {{27.5, 47.63, 0.0}},   // Yaw
                            {{33.25, 0.0, 0.0}},    // Pitch1
                            {{60.0, 0.0, 0.0}},     // Pitch2
                            {{120.0, 0.0, 0.0}},    // Tip
                            }};
    arm_config.model.axis = {{
                            {{0.0, 0.0, 1.0}},      // Yaw
                            {{0.0, 1.0, 0.0}},      // Pitch1
                            {{0.0, 1.0, 0.0}},      // Pitch2
                            {{0.0, 0.0, 0.0}},      // Tip
                            }};
    arm_config.model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};

    // LEFT FRONT
    arm_config.joint.q_min = {{0.0, -60.0, 0.0}};
    arm_config.joint.q_max = {{90.0, 75.0, 150.0}};
    arm_config.act.id = {{0, 1, 2}};
    arm_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    arm_config.act.offset = {{60.0, 0.0, -90.0}};
    config.arm_config[LEFT_FRONT] = arm_config;
    // LEFT BACK
    arm_config.joint.q_min = {{90.0, -60.0, 0.0}};
    arm_config.joint.q_max = {{180.0, 75.0, 150.0}};
    arm_config.act.id = {{3, 4, 5}};
    arm_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    arm_config.act.offset = {{120.0, 0.0, -90.0}};
    config.arm_config[LEFT_BACK] = arm_config;
    // RIGHT FRONT
    arm_config.joint.q_min = {{-90.0, -60.0, 0.0}};
    arm_config.joint.q_max = {{0.0, 75.0, 150.0}};
    arm_config.act.id = {{6, 7, 8}};
    arm_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    arm_config.act.offset = {{-60.0, 0.0, -90.0}};
    config.arm_config[RIGHT_FRONT] = arm_config;
    // RIGHT BACK
    arm_config.model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    arm_config.joint.q_min = {{-180.0, -60.0, 0.0}};
    arm_config.joint.q_max = {{-90.0, 75.0, 150.0}};
    arm_config.act.id = {{9, 10, 11}};
    arm_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
    arm_config.act.offset = {{-120.0, 0.0, -90.0}};
    config.arm_config[RIGHT_BACK] = arm_config;
}

void Robot::UpdateState(RobotState& state) {
    // body
    state.body.trans.translation() = Vector3d(0.0, 0.0, 0.0);
    //state.body.trans.linear() = MatrixFromRpy(rpy);
    // arm
    for (size_t i = 0; i < arm_.size(); i++) {
        arm_[i].ForwardKinematic(state.arm[i].q, state.body.trans, state.arm[i].trans);
    }
}

void Robot::UpdateRef(RobotOut& out) {
    for (size_t i = 0; i < arm_.size(); i++) {
        bool is_joint_limit = false;
        bool ik_ret = arm_[i].InverseKinematic(out.arm[i].trans, out.body.trans, is_joint_limit, out.arm[i].q);
    }
}

void Robot::ConvertInput(const InputState& input, RobotState& state) {
    for (size_t i = 0; i < arm_.size(); i++) {
        const ActConfig& act_config = config_.arm_config[i].act;
        for (size_t j = 0; j < act_config.id.size(); j++) {
        }
    }
}

void Robot::ConvertOutput(const RobotRef& ref, OutputState& output) {
    for (size_t i = 0; i < arm_.size(); i++) {
        const ActConfig& act_config = config_.arm_config[i].act;
        for (size_t j = 0; j < act_config.id.size(); j++) {
            const int index = i * act_config.id.size() + j;
            output.serial_servo[index].id = act_config.id[j];
            output.serial_servo[index].act_q = act_config.gain[j] * ref.arm[i].q[j] + act_config.offset[j];
            output.serial_servo[index].act_qd = 0.0;
            output.serial_servo[index].enable = true;
        }
    }
}

void Robot::GetDefaultRef(RobotRef& ref) {
    // body
    ref.body.trans = Affine3d::Identity();
    ref.body.trans.linear() = MatrixFromRpy(Vector3d(0.0, 0.0, 0.0));
    // arm
    for (size_t i = 0; i < ref.arm.size(); i++) {
        ref.arm[i].q = GetDefaultJoint(static_cast<ArmId>(i));
        arm_[i].ForwardKinematic(ref.arm[i].q, Affine3d::Identity(), ref.arm[i].trans);
    }
}

Vector3d Robot::GetDefaultJoint(const ArmId& id) {
    Vector3d q_deg = Vector3d::Zero();
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

/**
 * @class RobotMain
 */
RobotMain::RobotMain() {
    ESP_LOGI("Robot Main", "Constructor");
}

void RobotMain::Run() {
    ESP_LOGI("Robot Main", "Run");
    th_.Start(RobotMain::LaunchThread, "robot_thread", 2, 4096, &robot_, 0);
}

uint32_t RobotMain::StackMargin() {
    return th_.GetStackMargin();
}

void RobotMain::LaunchThread(void* arg) {
    ESP_LOGI("Robot Main", "Launch");
    reinterpret_cast<Robot*>(arg)->Thread();
}