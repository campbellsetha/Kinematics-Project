#include "wifi_init.h"
#include "esp_wifi.h"

esp_err_t wifi_init_default(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    return esp_wifi_init(&cfg);
}
