#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include <stdint.h>
#include <array>
#include <vector>
#include "driver/mcpwm.h"

struct PwmOutputConfig {
    mcpwm_unit_t unit;
    mcpwm_io_signals_t output;
};

struct PwmServoConfig {
    uint8_t pin;
    uint8_t pulse_period_ms;
    uint32_t pulse_min_width_us;
    uint32_t pulse_max_width_us;
    int8_t position_min_deg;
    int8_t position_max_deg;
    PwmOutputConfig output_config;
    uint8_t timer_index;
};

class PwmServo {
    public:
    PwmServo();
    bool Append(const PwmServoConfig& config);
    bool SetPosition(uint8_t pin, double deg);
    private:
    uint8_t FindIndexFromPin(uint8_t pin);
    uint32_t ConvertToPulse(double deg, const PwmServoConfig& config);
    uint8_t num_pwm_;
    std::vector<PwmServoConfig> config_;
    static const std::array<PwmOutputConfig, 12> pwm_output_config_map_; 
};

#endif