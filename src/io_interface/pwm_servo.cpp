#include "pwm_servo.hpp"

#include "soc/mcpwm_periph.h"
#include "esp_log.h"
/**
 * @brief Pwm output map
 */
const std::array<PwmOutputConfig, 12> PwmServo::pwm_output_config_map_ = {{
    {MCPWM_UNIT_0, MCPWM0A},
    {MCPWM_UNIT_0, MCPWM0B},
    {MCPWM_UNIT_0, MCPWM1A},
    {MCPWM_UNIT_0, MCPWM1B},
    {MCPWM_UNIT_0, MCPWM2A},
    {MCPWM_UNIT_0, MCPWM2B},
    {MCPWM_UNIT_1, MCPWM0A},
    {MCPWM_UNIT_1, MCPWM0B},
    {MCPWM_UNIT_1, MCPWM1A},
    {MCPWM_UNIT_1, MCPWM1B},
    {MCPWM_UNIT_1, MCPWM2A},
    {MCPWM_UNIT_1, MCPWM2B}
}};

/**
 * @class PwmServo
 */
PwmServo::PwmServo()
    :   num_pwm_(0) {}

bool PwmServo::Append(const PwmServoConfig& config) {
    if (num_pwm_ >= pwm_output_config_map_.size()) return false;

    PwmServoConfig pwm_servo_config = config;
    pwm_servo_config.output_config = pwm_output_config_map_[num_pwm_];
    
    // initialize
    mcpwm_gpio_init(pwm_servo_config.output_config.unit, pwm_servo_config.output_config.output, pwm_servo_config.pin);
    
    // set 
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000 / pwm_servo_config.pulse_period_ms;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(pwm_servo_config.output_config.unit, MCPWM_TIMER_0, &pwm_config);

    // update number of pwm servo
    config_.push_back(pwm_servo_config);
    num_pwm_++;

    return true;
}

bool PwmServo::SetPosition(uint8_t pin, double deg) {
    const int8_t index = FindIndexFromPin(pin);
    if (index < 0) return false;

    const PwmServoConfig& config = config_[index];
    mcpwm_generator_t generator = MCPWM_GEN_A;
    if (config.output_config.output == MCPWM0B || config.output_config.output == MCPWM1B || config.output_config.output == MCPWM2B ) {
        generator = MCPWM_GEN_B;
    }
    uint32_t pulse = ConvertToPulse(deg, config_[index]);
    mcpwm_set_duty_in_us(config.output_config.unit, MCPWM_TIMER_0, generator, pulse);
    
    return true;
}

uint8_t PwmServo::FindIndexFromPin(uint8_t pin) {
    for (size_t i = 0; i < config_.size(); i++) {
        if (config_[i].pin == pin) {
            return i;
        }
    }
    return -1;
}

uint32_t PwmServo::ConvertToPulse(double deg, const PwmServoConfig& config) {
    return (config.pulse_max_width_us - config.pulse_min_width_us)
            / (config.position_max_deg - config.position_min_deg)
             * (deg - config.position_min_deg) + config.pulse_min_width_us;
}