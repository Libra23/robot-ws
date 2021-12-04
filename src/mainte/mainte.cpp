#include "mainte.hpp"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <string.h>
#include "control_data/packet_data.hpp"
//#define MAINTE_DEBUG
#ifdef MAINTE_DEBUG
#define MAINTE_LOG(...) ESP_LOGI(__VA_ARGS__)
#define MAINTE_DEBUG_LOG(...) ESP_LOGI(__VA_ARGS__)
#else
#define MAINTE_LOG(...) ESP_LOGI(__VA_ARGS__)
#define MAINTE_DEBUG_LOG(...)
#endif
static const char *TAG = "Mainte";

#define PORT 3333

Maintenance::Maintenance() {
    MAINTE_LOG(TAG, "Constructor");
}

bool Maintenance::StartConnection() {
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

    // Create default loop for wifi event handler
    error = esp_event_loop_create_default();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to create default loop. error id = %d", error);
        return false;
    }

    // Create default WIFI AP
    p_netif_ = esp_netif_create_default_wifi_ap();

    // Initialize wifi
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    error = esp_wifi_init(&cfg);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to initialize wifi. error id = %d", error);
        return false;
    }

    // Register callback
    error = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &CallBackConnection, NULL, NULL);
    error |= esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &CallBackConnection, NULL, NULL);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to register callback function. error id = %d", error);
        return false;
    }

    // Set Mode
    error = esp_wifi_set_mode(WIFI_MODE_AP);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to set wifi mode. error id = %d", error);
        return false;
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
    }

    // Start
    error = esp_wifi_start();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to start wifi. error id = %d", error);
        return false;
    }

    MAINTE_LOG(TAG, "Complete to start.");
    return true;
}

bool Maintenance::StopConnection() {
    int error = 0;
    // Stop wifi
    error = esp_wifi_stop();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to stop wifi. error id = %d", error);
        return false;
    }
    esp_netif_destroy_default_wifi(p_netif_);

    // Delete wifi loop
    error = esp_event_loop_delete_default();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to delete wifi loop. error id = %d", error);
        return false;
    }

    // Deinit
    error = esp_netif_deinit();
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to deinitialize netif. error id = %d", error);
        return false;
    }

    MAINTE_LOG(TAG, "Complete to stop.");
    return true;
}

bool Maintenance::SetStaticIPAddress() {
    int error = 0;
    // Stop DHCP
    error = tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to stop dhcp. error id = %d", error);
        return false;
    }

    // Set IP
    tcpip_adapter_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 10,10,11,2);
    IP4_ADDR(&ip_info.gw, 10,10,11,1);
    IP4_ADDR(&ip_info.netmask, 255,255,255,0);
    error = tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to set IP. error id = %d", error);
        return false;
    }

    // Restart DHCP
    error = tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP);
    if (error != 0) {
        MAINTE_DEBUG_LOG(TAG, "Fail to start dhcp. error id = %d", error);
        return false;
    }

    // Check IP
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info);
    MAINTE_LOG(TAG, "Set static IP address Finished. IP = " IPSTR, IP2STR(&ip_info.ip));
    return true;
}

void Maintenance::CallBackConnection(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        MAINTE_LOG(TAG, "Station join =" MACSTR ", AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        MAINTE_LOG(TAG, "Station leave=" MACSTR ", AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        MAINTE_LOG(TAG, "WiFi ap start");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
        MAINTE_LOG(TAG, "WiFi ap stop");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED){
        ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*) event_data;
        MAINTE_LOG(TAG, "Set ip=" IPSTR, IP2STR(&event->ip));
    }
}

void Maintenance::Thread() {
    MAINTE_LOG(TAG, "Thread");
    bool is_ok = true;
    is_ok &= StartConnection();
    // is_ok &= SetStaticIPAddress();

    // Wait unitil establish connection
    esp_netif_sta_info_t station_info;
    while(true) {
        wifi_sta_list_t wifi_sta_list;
        esp_wifi_ap_get_sta_list(&wifi_sta_list);
        tcpip_adapter_sta_list_t station_list;
        tcpip_adapter_get_sta_list(&wifi_sta_list, &station_list);
        
        if (station_list.num > 0) {
            station_info = station_list.sta[0];
            MAINTE_LOG(TAG, "Station Address : " IPSTR " Mac Address : " MACSTR, IP2STR(&station_info.ip), MAC2STR(station_info.mac));
            break;
        }
        delay(1000);
    }

    // Set mainte server ip
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = station_info.ip.addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    std::array<uint8_t, 2048> rx_buffer;
    while(true) {
        // Prepare socket
        int sock =  socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            MAINTE_LOG(TAG, "Unable to create socket. errno id =  %d", errno);
            break;
        }
        MAINTE_LOG(TAG, "Socket created, connecting to host.");

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(sockaddr_in));
        if (err != 0) {
            MAINTE_LOG(TAG, "Socket unable to connect. errno id = %d", errno);
            break;
        }
        MAINTE_LOG(TAG, "Successfully connect to Host.");

        while(true) {
            PacketRobotInfoRes robot_info_req;
            int err = send(sock, &robot_info_req, sizeof(PacketRobotInfoRes), 0);
            if (err < 0) {
                MAINTE_LOG(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rx_buffer.data(), rx_buffer.size(), 0);
            // Error occurred during receiving
            if (len < 0) {
                MAINTE_LOG(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                MAINTE_LOG(TAG, "Received %d bytes", len);
                uint8_t tcp_packet_type = GetPacketType(rx_buffer.data());
                if (tcp_packet_type == MAINTE_TO_ROBOT_CONTROL_DATA) {
                    PacketControlDataReq control_data_req;
                    memmove(&control_data_req, rx_buffer.data(), sizeof(control_data_req));
                    MAINTE_LOG(TAG, "arm_id = %d", control_data_req.arm_id);
                }
            }
            delay(10000);
        }

        if (sock != -1) {
            MAINTE_LOG(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }

    StopConnection();
    MAINTE_LOG(TAG, "Kill Task");
    vTaskDelete(NULL);
}

/**
 * @class MaintenanceMain
 */
MaintenanceMain::MaintenanceMain() {
    MAINTE_LOG("Maintenance Main", "Constructor");
}

void MaintenanceMain::Run() {
    MAINTE_LOG("Maintenance Main", "Run");
    th_.Start(MaintenanceMain::LaunchThread, "mainte_thread", 1, 8192, &maintenance_, 1);
}

uint32_t MaintenanceMain::StackMargin() {
    return th_.GetStackMargin();
}

void MaintenanceMain::LaunchThread(void* arg) {
    MAINTE_LOG("Maintenance Main", "Launch");
    reinterpret_cast<Maintenance*>(arg)->Thread();
}