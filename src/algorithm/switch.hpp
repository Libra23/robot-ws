#ifndef SWITCH_H
#define SWITCH_H

#define PUSH_SHORT_MS 1000
#define PUSH_LONG_MS 3000
#define DETECT_TIME_MS 100
#define SECOND_TO_MICROSECOND 1000

enum SwitchStatus {
    RELEASE,
    PUSH,
    SHORT_PUSHED,
    LONG_PUSHING,
    LONG_PUSHED,
    SHORT_RELEASED,
    LONG_RELEASED,
    TRIGGER_UP,
    TRIGGER_DOWN
};

class Switch {
    public:
    Switch();
    void Config(bool push_status);
    bool Update(double time, bool raw_status);
    SwitchStatus GetStatus() const;
    private:
    bool push_status_;
    SwitchStatus pre_status_;
    double detect_time;
};

#endif