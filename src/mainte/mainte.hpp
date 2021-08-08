#include "common/thread.hpp"
#include "freertos/event_groups.h"
#include "esp_event.h"
class Maintenance {
    public:
    Maintenance();
    bool Initialize();
    static void CallBack(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
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