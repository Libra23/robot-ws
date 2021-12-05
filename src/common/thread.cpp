#include "thread.hpp"
#include "esp_timer.h"

Thread::Thread() {

}

void Thread::Start(void (*function)(void*), const std::string& name, uint8_t priority, uint32_t stack_size, void* arg, uint8_t core) {
    xTaskCreatePinnedToCore(function, name.c_str(), stack_size, arg, priority, &task_handle_, core);
}

uint32_t Thread::GetStackMargin() {
    return uxTaskGetStackHighWaterMark(task_handle_);
}

void delay(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

int64_t get_time_us() {
    return esp_timer_get_time();
}

int64_t get_time_ms() {
    return get_time_us() * 1000;
}
