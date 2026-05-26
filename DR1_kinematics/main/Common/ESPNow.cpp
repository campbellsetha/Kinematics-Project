#include <cstdio>
#include <cstring>
#include <cinttypes>
#include "ESPNow.hpp"

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_event.h"
    #include "esp_netif.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "wifi_init.h"
}

uint8_t MACAddressArray[3][6] = {
    {0x10, 0x00, 0x3b, 0xcd, 0x55, 0xc8},
    {0x10, 0x00, 0x3b, 0xcb, 0x96, 0xa8},
    {0x10, 0x00, 0x3b, 0xcb, 0xe4, 0xac},
};

uint8_t ownMacAddress[6];

static void (*g_positionsReceivedCb)(ROLE, const MSGALLPOSITIONS*) = nullptr;

static const char* TAG = "ESPNow";

void ESPNow::init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();

    ret = wifi_init_default();
    ESP_LOGI(TAG, "esp_wifi_init:     %s", esp_err_to_name(ret));

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    ESP_LOGI(TAG, "esp_wifi_set_mode: %s", esp_err_to_name(ret));

    ret = esp_wifi_start();
    ESP_LOGI(TAG, "esp_wifi_start:    %s", esp_err_to_name(ret));

    esp_wifi_get_mac(WIFI_IF_STA, ownMacAddress);

    ret = esp_now_init();
    ESP_LOGI(TAG, "esp_now_init:      %s", esp_err_to_name(ret));

    esp_now_register_recv_cb(ESPNow::readMessage);

    addPeers();
}

void ESPNow::addPeers() {
    for (int i = 0; i < PEER_COUNT; i++) {
        if (memcmp(MACAddressArray[i], ownMacAddress, 6) != 0) {
            esp_now_peer_info_t peer{};
            memcpy(peer.peer_addr, MACAddressArray[i], 6);
            peer.channel = 0;
            peer.encrypt = false;
            peer.ifidx   = WIFI_IF_STA;
            esp_now_add_peer(&peer);
        }
    }
}

void ESPNow::sendMessage(ROLE role, const uint8_t* data, size_t len) {
    const uint8_t* addr = nullptr;
    switch (role) {
        case PRIMARY:  addr = MACAddressArray[0]; break;
        case LEADER:   addr = MACAddressArray[1]; break;
        case FOLLOWER: addr = MACAddressArray[2]; break;
        default: return;
    }
    static uint32_t s_send_count = 0;
    esp_err_t ret = esp_now_send(addr, data, len);
    ++s_send_count;
    if (s_send_count <= 3 || s_send_count % 100 == 0)
        ESP_LOGI(TAG, "send #%" PRIu32 " role=%d: %s", s_send_count, (int)role, esp_err_to_name(ret));
}

void ESPNow::setPositionsCallback(void (*cb)(ROLE, const MSGALLPOSITIONS*)) {
    g_positionsReceivedCb = cb;
}

void ESPNow::readMessage(const esp_now_recv_info_t* info, const uint8_t* data, int data_len) {
    if (data_len < 1) return;

    switch (static_cast<MSG_TYPE>(data[0])) {
        case MODE_OF_OPERATION: {
            if (data_len < (int)sizeof(MSGOPERATIONMODE)) return;
            break;
        }
        case COORDINATES: {
            if (data_len < (int)sizeof(MSGCOORDINATES)) return;
            break;
        }
        case ALL_POSITIONS: {
            if (data_len < (int)sizeof(MSGALLPOSITIONS)) return;
            const MSGALLPOSITIONS* msg = reinterpret_cast<const MSGALLPOSITIONS*>(data);
            if (g_positionsReceivedCb) {
                ROLE src = PRIMARY;
                for (int i = 0; i < (int)PEER_COUNT; i++) {
                    if (memcmp(info->src_addr, MACAddressArray[i], 6) == 0) {
                        src = static_cast<ROLE>(i + 1);
                        break;
                    }
                }
                g_positionsReceivedCb(src, msg);
            }
            break;
        }
    }
}
