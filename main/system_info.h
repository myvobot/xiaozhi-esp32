#ifndef _SYSTEM_INFO_H_
#define _SYSTEM_INFO_H_

#include <string>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

class SystemInfo {
public:
    static size_t GetFlashSize();
    static size_t GetMinimumFreeHeapSize();
    static size_t GetFreeHeapSize();
#if CONFIG_BOARD_TYPE_VOBOT_GLOBAL_ESP32S3
    static std::string GetDeviceId();
#endif
    static std::string GetMacAddress();
    static std::string GetChipModelName();
    static esp_err_t PrintRealTimeStats(TickType_t xTicksToWait);
};

#endif // _SYSTEM_INFO_H_
