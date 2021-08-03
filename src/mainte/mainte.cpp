#include "mainte.hpp"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
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

    // Create default WIFI STA.
    esp_netif_create_default_wifi_sta();

    // Create default loop for wifi event handler
    error = esp_event_loop_create_default();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to create default loop. error id = %d", error);
        return false;
    }

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    // Regist instance of event handler to default loop
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_init(&WIFI_INIT_CONFIG_DEFAULT()));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    MAINTE_LOG(TAG, "wifi_init_sta finished.");
    
    s_wifi_event_group_ = xEventGroupCreate();

}

void Maintenance::Thread() {

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