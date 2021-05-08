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
        arm_[i].Config(config_.arm_config[i], i);
    }

    int count_ms = 0;
    while(true) {
        // input
        InputState input;
        input_memory_.Read(input);

        // state
        RobotState state;
        ConvertInput(input, state);

        // calcurate trans
        for (size_t i = 0; i < arm_.size(); i++) {
            // convert act_q to q
            arm_[i].ConvertToJoint(state.arm[i].act_q, state.arm[i].q);

            // convert q to trans
            arm_[i].ForwardKinematic(state.arm[i].q, state.body.trans, state.arm[i].trans);
        }
        
        // ref
        RobotRef ref;
        GetDefaultRef(ref);

        // calculate q & act_q
        for (size_t i = 0; i < arm_.size(); i++) {
            bool is_limit;
            // convert trans to q
            arm_[i].InverseKinematic(ref.arm[i].trans, ref.body.trans, is_limit, ref.arm[i].q);

            // convert q to act_q
            arm_[i].ConvertToAct(ref.arm[i].q, ref.arm[i].act_q);
            
            //ESP_LOGI("Robot", "Leg%d : Pos xyz = %f, %f, %f\n", i, ref.arm[i].trans.translation()[X], ref.arm[i].trans.translation()[Y], ref.arm[i].trans.translation()[Z]);
            //ESP_LOGI("Robot", "Leg%d : Joint q = %f, %f, %f\n", i, ref.arm[i].q[0] * RAD_TO_DEG, ref.arm[i].q[1] * RAD_TO_DEG, ref.arm[i].q[2] * RAD_TO_DEG);
            //ESP_LOGI("Robot", "Leg%d : Joint act_q = %f, %f, %f\n", i, ref.arm[i].act_q[0], ref.arm[i].act_q[1], ref.arm[i].act_q[2]);
        }

        // output
        OutputState output;
        ConvertOutput(ref, output);
        output_memory_.Write(output);
        delay(10);
        //count_ms += 10;
    }
}

void Robot::CreateConfig(RobotConfig& config) {
    config.cog = {{0.0, 0.0, 0.0}};
    // arm model
    ArmConfig arm_config;

    if (IsQuadDiagonal()) {
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
        arm_config.joint.q_min = {{-180.0, -60.0, 0.0}};
        arm_config.joint.q_max = {{-90.0, 75.0, 150.0}};
        arm_config.act.id = {{9, 10, 11}};
        arm_config.act.gain = {{-RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG}};
        arm_config.act.offset = {{-120.0, 0.0, -90.0}};
        config.arm_config[RIGHT_BACK] = arm_config;

    } else if (IsQuadPupper()) {
        arm_config.model.axis = {{
                                {{1.0, 0.0, 0.0}},      // Roll
                                {{0.0, 1.0, 0.0}},      // Pitch1
                                {{0.0, 1.0, 0.0}},      // Pitch2
                                {{1.0, 0.0, 0.0}},      // Tip
                                }};
        arm_config.model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};

        // LEFT FRONT
        arm_config.model.xyz =  {{
                                {{54.75, 36.25, 0.0}},  // Roll
                                {{0.0, 39.0, 0.0}},     // Pitch1
                                {{0.0, 0.0, -60.0}},    // Pitch2
                                {{0.0, 0.0, -60.0}},    // Tip
                                }};
        arm_config.act.id = {{-1, -1, -1}};
        arm_config.act.gain = {{-RAD_TO_DEG, 0, 0, 
                                0, RAD_TO_DEG, 0,
                                0, -RAD_TO_DEG, -RAD_TO_DEG}};
        arm_config.act.offset = {{60.0, 0.0, -90.0}};
        config.arm_config[LEFT_FRONT] = arm_config;
        // LEFT BACK
        arm_config.model.xyz =  {{
                                {{-54.75, 36.25, 0.0}},  // Roll
                                {{0.0, 39.0, 0.0}},      // Pitch1
                                {{0.0, 0.0, -60.0}},     // Pitch2
                                {{0.0, 0.0, -60.0}},     // Tip
                                }};
        arm_config.act.id = {{-1, -1, -1}};
        arm_config.act.gain = {{-RAD_TO_DEG, 0, 0, 
                                0, RAD_TO_DEG, 0,
                                0, -RAD_TO_DEG, -RAD_TO_DEG}};
        arm_config.act.offset = {{120.0, 0.0, -90.0}};
        config.arm_config[LEFT_BACK] = arm_config;
        // RIGHT FRONT
        arm_config.model.xyz =  {{
                                {{54.75, -36.25, 0.0}}, // Roll
                                {{0.0, -39.0, 0.0}},    // Pitch1
                                {{0.0, 0.0, -60.0}},    // Pitch2
                                {{0.0, 0.0, -60.0}},    // Tip
                                }};
        arm_config.joint.q_min = {{-90.0, -180.0, -180.0}};
        arm_config.joint.q_max = {{90.0, 180.0, 180.0}};
        arm_config.act.id = {{6, 7, 8}};
        arm_config.act.gain = {{-RAD_TO_DEG, 0, 0, 
                                0, RAD_TO_DEG, 0,
                                0, -RAD_TO_DEG, -RAD_TO_DEG}};
        arm_config.act.offset = {{0.0, -90.0, -90.0}};
        config.arm_config[RIGHT_FRONT] = arm_config;
        // RIGHT BACK
        arm_config.model.xyz =  {{
                                {{-54.75, -36.25, 0.0}}, // Roll
                                {{0.0, -39.0, 0.0}},     // Pitch1
                                {{0.0, 0.0, -60.0}},     // Pitch2
                                {{0.0, 0.0, -60.0}},     // Tip
                                }};
        arm_config.act.id = {{-1, -1, -1}};
        arm_config.act.gain = {{-RAD_TO_DEG, 0, 0, 
                                0, RAD_TO_DEG, 0,
                                0, -RAD_TO_DEG, -RAD_TO_DEG}};
        arm_config.act.offset = {{-120.0, 0.0, -90.0}};
        config.arm_config[RIGHT_BACK] = arm_config;
    }
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
            output.serial_servo[index].act_q = ref.arm[i].act_q[j];
            output.serial_servo[index].act_qd = 0.0;
            output.serial_servo[index].enable = true;
        }
    }
}

void Robot::GetDefaultRef(RobotRef& ref) {
    // body
    ref.body.trans = Affine3d::Identity();
    // arm
    for (size_t i = 0; i < ref.arm.size(); i++) {
        arm_[i].GetDefault(i, ref.arm[i].q, ref.body.trans, ref.arm[i].trans);
    }
}

/**
 * @class RobotMain
 */
RobotMain::RobotMain() {
    ESP_LOGI("Robot Main", "Constructor");
}

void RobotMain::Run() {
    ESP_LOGI("Robot Main", "Run");
    th_.Start(RobotMain::LaunchThread, "robot_thread", 2, 8192, &robot_, 0);
}

uint32_t RobotMain::StackMargin() {
    return th_.GetStackMargin();
}

void RobotMain::LaunchThread(void* arg) {
    ESP_LOGI("Robot Main", "Launch");
    reinterpret_cast<Robot*>(arg)->Thread();
}