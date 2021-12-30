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

ThreadClock::ThreadClock(uint32_t ms) {
    previous_wake_time_ = xTaskGetTickCount();
    wait_ms_ = ms;
}

void ThreadClock::Wait() {
    vTaskDelayUntil(&previous_wake_time_, wait_ms_ / portTICK_RATE_MS);
}

void ThreadClock::Reset() {
    previous_wake_time_ = xTaskGetTickCount();
}

void delay(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

int64_t get_time_us() {
    return esp_timer_get_time();
}

int64_t get_time_ms() {
    return static_cast<int64_t>(get_time_us() * 0.001);
}
