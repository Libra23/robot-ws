#ifndef WAVE_H
#define WAVE_H

enum WaveType {
    CONST,
    SIN,
    RECTANGLE,
    TRIANGLE,
};

struct WaveForm {
    WaveType type;
    double amplitude;
    double base;
    double frequency;
    double phase;
};

class Wave {
    public:
    Wave();
    void Config(const WaveForm& form);
    void Config(WaveType type, double amplitude, double base, double frequency, double phase);
    double Update(double time);
    private:
    WaveForm form_;
};

#endif