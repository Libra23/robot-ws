#include "robot_control.hpp"
#include "esp_log.h"

#include "common/extern_definition.hpp"
#include "constant/clock_const.hpp"

//#define ROBOT_CONTROL_DEBUG
#ifdef ROBOT_CONTROL_DEBUG
#define ROBOT_LOG(...) ESP_LOGI(__VA_ARGS__)
#define ROBOT_DEBUG_LOG(...) ESP_LOGI(__VA_ARGS__)
#else
#define ROBOT_LOG(...) ESP_LOGI(__VA_ARGS__)
#define ROBOT_DEBUG_LOG(...)
#endif
static const char *TAG = "Robot";

/**
 * @class Robot
 */
Robot::Robot() :
    clock_(ROBOT_CONTROL_CYCLE_TIME_MS)  {
    ROBOT_LOG(TAG, "Call Constructor");
}

void Robot::Thread() {
    ROBOT_LOG(TAG, "Start Task");
    Initialize();
    // main loop
    while(true) {
        if (true) {
            // synchronize with semaphore
            if(!sync_semaphore_.Take()) {
                break;
            }
        } else {
            // synchronize with cycle period
            clock_.Wait();
        }
        // main
        Excute();
        // update
        counter_++;
        control_time_ = counter_ * ROBOT_CONTROL_CYCLE_TIME_MS * MS_TO_S;
    }
    ROBOT_LOG(TAG, "Kill Task");
    vTaskDelete(NULL);
}

void Robot::Initialize() {
    // set config
    CreateConfig(config_);
    for (size_t i = 0; i < arm_.size(); i++) {
        arm_[i].Config(config_.arm_config[i], i);
        motion_[i].reset(new FKGenerator(NUM_JOINT));
    }
    clock_.Reset();
    counter_ = 0;
    control_time_ = 0.0;
    GetDefaultRef(ref_pre_);
}

void Robot::Excute() {
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

    // react msg from mainte server
    ReactReceivedMsg();

    // ref
    RobotRef ref;
    ref.time = get_time_ms();
    for (size_t i = 0; i < arm_.size(); i++) {
        // update ref
        motion_[i]->Update(control_time_, ref_pre_.arm[i].q, ref.arm[i].q);
        // convert q to act_q
        arm_[i].ConvertToAct(ref.arm[i].q, ref.arm[i].act_q);
    }

    // output
    OutputState output;
    ConvertOutput(ref, output);
    output_memory_.Write(output);

    // monitor
    const int skip = (1 * S_TO_MS / ROBOT_CONTROL_CYCLE_TIME_MS); // 1 s
    if (counter_ % skip == 0) {
        // const int index = RIGHT_BACK;
        // ROBOT_LOG(TAG, "count = %lld, act_q = (%f, %f, %f)", counter_, ref.arm[index].act_q[0], ref.arm[index].act_q[1], ref.arm[index].act_q[2]);
        // const uint64_t delta_time = (ref.time - ref_pre_.time);
        // ROBOT_LOG(TAG, "counter = %lld, control_time = %f, delta_time_ms = %lld", counter_, control_time_,  delta_time);
    }
    ref_pre_ = ref;
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
        arm_config.joint.q_min = {{-90.0, -180.0, -180.0}};
        arm_config.joint.q_max = {{90.0, 180.0, 180.0}};
        arm_config.act.id = {{0, 1, 2}};
        arm_config.act.gain = {{-RAD_TO_DEG, 0, 0, 
                                0, -RAD_TO_DEG, 0,
                                0, RAD_TO_DEG, RAD_TO_DEG}};
        arm_config.act.offset = {{0.0, 90.0, 90.0}};
        config.arm_config[LEFT_FRONT] = arm_config;
        // LEFT BACK
        arm_config.model.xyz =  {{
                                {{-54.75, 36.25, 0.0}},  // Roll
                                {{0.0, 39.0, 0.0}},      // Pitch1
                                {{0.0, 0.0, -60.0}},     // Pitch2
                                {{0.0, 0.0, -60.0}},     // Tip
                                }};
        arm_config.act.id = {{3, 4, 5}};
        arm_config.act.gain = {{RAD_TO_DEG, 0, 0, 
                                0, -RAD_TO_DEG, 0,
                                0, RAD_TO_DEG, RAD_TO_DEG}};
        arm_config.act.offset = {{0.0, 90.0, 90.0}};
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
        arm_config.act.id = {{9, 10, 11}};
        arm_config.act.gain = {{RAD_TO_DEG, 0, 0, 
                                0, RAD_TO_DEG, 0,
                                0, -RAD_TO_DEG, -RAD_TO_DEG}};
        arm_config.act.offset = {{0.0, -90.0, -90.0}};
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
        arm_[i].InverseKinematic(out.arm[i].trans, out.body.trans, is_joint_limit, out.arm[i].q);
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

void Robot::ReactReceivedMsg() {
    const uint32_t num_of_msg = mainte_to_robot_queue_.NumOfItems();
    for (uint32_t i = 0; i < num_of_msg; i++) {
        uint8_t buf[GetMaxMsgSize()];
        mainte_to_robot_queue_.Receive(buf);
        uint8_t type = GetMsgType(buf);
        ROBOT_LOG(TAG, "Received Type = %d", type);

        switch(type) {
            case MSG_MAINTE_TO_ROBOT_CONTROL_ON: {
                MsgCmdControl cmd(buf);
                ReactControlOn(cmd.arm_id, cmd.control_data);
                break;
            }
            case MSG_MAINTE_TO_ROBOT_CONTROL_OFF: {
                MsgCmdControl cmd(buf);
                ReactControlOff(cmd.arm_id);
                break;
            }
            default: {
                break;
            }

        }
    }
}

void Robot::ReactControlOn(int arm_id, const ControlData& control_data) {
    ROBOT_LOG(TAG, "Call React Control On, arm : %d, mode : %d", arm_id, control_data.control_mode);
    // update control mode
    switch(control_data.control_mode) {
        case ControlMode::FK: {
            motion_[arm_id].reset(new FKGenerator(NUM_JOINT));
            break;
        }
        case ControlMode::IK: {
            // motion_[arm_id].reset(new IKGenerator(arm_[i]));
            break;
        }
        case ControlMode::ACT_FK: {
            break;
        }
    }
    // update config
    motion_[arm_id]->Config(control_data.reference);
    motion_[arm_id]->CompleteConfig();
    // start
    motion_[arm_id]->StartRequest();
}

void Robot::ReactControlOff(int arm_id) {
    // stop
    motion_[arm_id]->StopRequest();
}

/**
 * @class RobotMain
 */
RobotMain::RobotMain() {
    ROBOT_LOG("Robot Main", "Constructor");
}

void RobotMain::Run() {
    ROBOT_LOG("Robot Main", "Run");
    th_.Start(RobotMain::LaunchThread, "robot_thread", 10, 8192, &robot_, 0);
}

uint32_t RobotMain::StackMargin() {
    return th_.GetStackMargin();
}

void RobotMain::LaunchThread(void* arg) {
    ROBOT_LOG("Robot Main", "Launch");
    reinterpret_cast<Robot*>(arg)->Thread();
}