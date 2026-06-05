// Author: Seth Campbell
// app_main.cpp
// Firmware entry point — reads this board's MAC address and starts exactly one
// role (Primary, Leader, or Follower) based on which MAC matches the known list.
// All three roles share the same binary; the MAC determines behaviour at boot.

#include <cstring>
#include "Common/ESPNow.hpp"
#include "Leader/Leader.hpp"
#include "Follower/Follower.hpp"
#include "Primary/Primary.hpp"

extern "C" {
    #include "esp_mac.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

static const char* TAG = "app_main";

static Leader   g_leader;
static Follower g_follower;
static Primary  g_primary;

extern "C" void app_main(void) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Match this board's MAC against the known address table and boot the correct role
    if (memcmp(mac, mac_address_array[0], 6) == 0) {
        ESP_LOGI(TAG, "Role: PRIMARY");
        g_primary.init();
    } else if (memcmp(mac, mac_address_array[1], 6) == 0) {
        ESP_LOGI(TAG, "Role: LEADER");
        g_leader.init();
    } else if (memcmp(mac, mac_address_array[2], 6) == 0) {
        ESP_LOGI(TAG, "Role: FOLLOWER");
        g_follower.init();
    } else {
        ESP_LOGE(TAG, "MAC does not match any known role — running idle");
    }

    // All work happens in FreeRTOS tasks; keep app_main alive with a low-priority idle loop
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}