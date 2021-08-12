#include "common/thread.hpp"
#include "freertos/event_groups.h"
#include "esp_event.h"
class Maintenance {
    public:
    Maintenance();
    bool StartConnection();
    bool StopConnection();
    bool SetStaticIPAddress();
    static void CallBackConnection(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    void Thread();
    private:
    esp_netif_t* p_netif_;
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