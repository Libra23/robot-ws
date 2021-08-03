#include "wave.hpp"
#include "math_const.hpp"
#include "math_utility.hpp"

Wave::Wave() {

}

void Wave::Config(const WaveForm& form) {
    form_ = form;
}

void Wave::Config(WaveType type, double amplitude, double base, double frequency, double phase) {
    form_.type = type;
    form_.amplitude = amplitude;
    form_.base = base;
    form_.frequency = frequency;
    form_.phase = phase * DEG_TO_RAD;
}

double Wave::Update(double time) {
    double ret = form_.base;
    const double ratio = fmod(time * form_.frequency, 1.0);
    switch(form_.type) {
        case WaveType::CONST:
        ret += 0.0;
        break;
        case WaveType::SIN:
        ret += form_.amplitude * sin(2 * PI * form_.frequency * time + form_.phase);
        break;
        case WaveType::RECTANGLE:
        if (ratio < 0.5) {
            ret += form_.amplitude;
        } else {
            ret += -form_.amplitude;
        }
        break;
        case WaveType::TRIANGLE:
        if (ratio < 0.5) {
            ret += 2 * form_.amplitude * ratio;
        } else {
            ret += 2 * form_.amplitude * (ratio - 1);
        }
        break;
        default:
        break;
    }
    return ret;
}