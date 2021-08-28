#ifndef WAVE_H
#define WAVE_H

enum WaveType {
    CONST,
    SIN,
    RECTANGLE,
    TRIANGLE,
};

#pragma pack(push, 1)
struct WaveForm {
    int type;
    double amplitude;
    double base;
    double frequency;
    double phase;
};
#pragma pack(pop)

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