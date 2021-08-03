#include "common/thread.hpp"
#include "freertos/event_groups.h"

class Maintenance {
    public:
    Maintenance();
    bool Initialize();
    void Thread();
    private:
    EventGroupHandle_t s_wifi_event_group_;
};

class MaintenanceMain {
    public:
    MaintenanceMain();
    void Run();
    uint32_t StackMargin();
    private:
    static void LaunchThread(void* arg);
    Maintenance maintenance_;
    Thread th_;
};