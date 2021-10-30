#include "switch.hpp"

Switch::Switch()
    :   push_status_(true),
        pre_status_(SwitchStatus::RELEASE) {

}

void Switch::Config(bool push_status) {
    push_status_ = push_status;
}

bool Switch::Update(double time, bool switch_raw_status) {
    // decide switch is pushed or released
    SwitchStatus status = (switch_raw_status == push_status_) ? SwitchStatus::PUSH : SwitchStatus::RELEASE;
    const double delta_ms = (time - detect_time) * SECOND_TO_MICROSECOND;
    if (pre_status_ != status) {
        if (delta_ms > DETECT_TIME_MS) {
            pre_status_ = status;
            return true;
        }
    } else {
        detect_time = time;
    }
    return false;
}

SwitchStatus Switch::GetStatus() const {
  return pre_status_;
}