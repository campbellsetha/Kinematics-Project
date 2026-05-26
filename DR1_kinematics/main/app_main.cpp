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

    if (memcmp(mac, MACAddressArray[0], 6) == 0) {
        ESP_LOGI(TAG, "Role: PRIMARY");
        g_primary.init();
    } else if (memcmp(mac, MACAddressArray[1], 6) == 0) {
        ESP_LOGI(TAG, "Role: LEADER");
        g_leader.init();
    } else if (memcmp(mac, MACAddressArray[2], 6) == 0) {
        ESP_LOGI(TAG, "Role: FOLLOWER");
        g_follower.init();
    } else {
        ESP_LOGE(TAG, "MAC does not match any known role — running idle");
    }

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
