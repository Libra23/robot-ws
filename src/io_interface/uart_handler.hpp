#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <stdint.h>
#include <string>
#include <vector>

enum UartParity {
    NONE,
    EVEN,
    ODD
};

struct UartConfig {
    uint8_t port_id;
    int8_t tx_pin;
    int8_t rx_pin;
    uint32_t buffer_size;
    uint32_t baud_rate;
    UartParity parity;
    uint32_t time_out_ms;
    UartConfig() : 
        port_id(0),
        tx_pin(-1),
        rx_pin(-1),
        buffer_size(256),
        baud_rate(115200),
        parity(UartParity::NONE),
        time_out_ms(1000) {}
};

class UartHandler{
    public:
    UartHandler();
    void Config(const UartConfig& config);
    void WriteBytes(const std::vector<uint8_t>& buf);
    void WriteString(const std::string& str);
    bool ReadBytes(std::vector<uint8_t>& buf);
    void FlushReceive();
    void WaitTransmit();
    int Available();
    private:
    int8_t port_id_;
    uint32_t buffer_size_;
    uint32_t time_out_ms_;
};

#endif