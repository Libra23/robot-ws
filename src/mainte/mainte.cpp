#include "mainte.hpp"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <string.h>
//#define MAINTE_DEBUG
#ifdef MAINTE_DEBUG
#define MAINTE_LOG(...) ESP_LOGI(__VA_ARGS__)
#define MAINTE_DEBUG_LOG(...) ESP_LOGI(__VA_ARGS__)
#else
#define MAINTE_LOG(...) ESP_LOGI(__VA_ARGS__)
#define MAINTE_DEBUG_LOG(...)
#endif
static const char *TAG = "Mainte";

Maintenance::Maintenance() {
    MAINTE_LOG(TAG, "Constructor");
}

bool Maintenance::Initialize() {
    int error = 0;
    // Init NVS for wifi
    error = nvs_flash_init();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to initialize nsv. error id = %d. Try to erase nvs.", error);
        error = nvs_flash_erase();
        if (error != 0) {
            MAINTE_DEBUG_LOG(TAG, "Fail to erase nsv. error id = %d.", error);
            return false;
        } else {
            error = nvs_flash_init();
            if (error != 0) {
                MAINTE_DEBUG_LOG(TAG, "Fail to initialize nsv again. error id = %d", error);
                return false;
            }
        }
    }
    MAINTE_DEBUG_LOG(TAG, "Complete to initialize nsv");

    // Init stack for TCP/IP
    error = esp_netif_init();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to initialize stack. error id = %d", error);
        return false;
    }
    MAINTE_DEBUG_LOG(TAG, "Complete to prepare stack");

    // Create default WIFI AP
    esp_netif_create_default_wifi_ap();

    // Create default loop for wifi event handler
    error = esp_event_loop_create_default();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to create default loop. error id = %d", error);
        return false;
    } else {
        MAINTE_DEBUG_LOG(TAG, "Complete to create default loop");
    }

    error = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &CallBack, NULL, NULL);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to register callback function. error id = %d", error);
        return false;
    } else {
        MAINTE_DEBUG_LOG(TAG, "Complete to register callback function");
    }

    // Initialize wifi
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    error = esp_wifi_init(&cfg);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to initialize wifi. error id = %d", error);
        return false;
    } else {
        MAINTE_DEBUG_LOG(TAG, "Complete to initialize wifi");
    }

    // Set Mode
    error = esp_wifi_set_mode(WIFI_MODE_AP);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to set wifi mode. error id = %d", error);
        return false;
    } else {
        MAINTE_DEBUG_LOG(TAG, "Complete to set wifi mode.");
    }

    // Set config
    wifi_config_t wifi_config;
    strcpy((char*)(wifi_config.ap.ssid), "esp-ssid");
    wifi_config.ap.ssid_len = strlen("esp-ssid");
    strcpy((char*)(wifi_config.ap.password), "mypassword");
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    error = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to set wifi config. error id = %d", error);
        return false;
    } else {
        MAINTE_DEBUG_LOG(TAG, "Complete to set wifi config wifi.");
    }

    // Start
    error = esp_wifi_start();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to start wifi. error id = %d", error);
        return false;
    } else {
        MAINTE_DEBUG_LOG(TAG, "Start wifi");
    }
    MAINTE_LOG(TAG, "wifi_init_sta finished.");
    
    // s_wifi_event_group_ = xEventGroupCreate();

    return true;
}

void Maintenance::CallBack(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        MAINTE_LOG(TAG, "station join, AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        MAINTE_LOG(TAG, "station leave, AID=%d", event->aid);
    }
}

void Maintenance::Thread() {
    MAINTE_LOG(TAG, "Thread");
    Initialize();
    while(true) {
        delay(10);
    }
}

/**
 * @class MaintenanceMain
 */
MaintenanceMain::MaintenanceMain() {
    MAINTE_LOG("Maintenance Main", "Constructor");
}

void MaintenanceMain::Run() {
    MAINTE_LOG("Maintenance Main", "Run");
    th_.Start(MaintenanceMain::LaunchThread, "mainte_thread", 1, 4096, &maintenance_, 1);
}

uint32_t MaintenanceMain::StackMargin() {
    return th_.GetStackMargin();
}

void MaintenanceMain::LaunchThread(void* arg) {
    MAINTE_LOG("Maintenance Main", "Launch");
    reinterpret_cast<Maintenance*>(arg)->Thread();
}