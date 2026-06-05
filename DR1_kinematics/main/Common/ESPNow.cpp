// Author: Seth Campbell
// ESPNow.cpp
// Implements wireless communication between boards using ESP-NOW.
// Handles initialisation, peer registration, sending, and dispatching received packets.

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

// Known MAC addresses for all three boards — order matches the ROLE enum (PRIMARY, LEADER, FOLLOWER)
uint8_t mac_address_array[3][6] = {
    {0x10, 0x00, 0x3b, 0xcd, 0x55, 0xc8},
    {0x10, 0x00, 0x3b, 0xcb, 0x96, 0xa8},
    {0xac, 0x27, 0x6e, 0x5e, 0x6b, 0x74},
};

uint8_t own_mac_address[6]; // Populated at init from the hardware

// User-supplied callbacks — set via set_*_callback() before calling init()
static void (*g_positions_received_cb)(ROLE, const MSGALLPOSITIONS*) = nullptr;
static void (*g_mode_received_cb)(OperationMode) = nullptr;

static const char* TAG = "ESPNow";

// Starts WiFi in station mode, brings up ESP-NOW, and registers all peers.
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

    esp_wifi_get_mac(WIFI_IF_STA, own_mac_address);

    ret = esp_now_init();
    ESP_LOGI(TAG, "esp_now_init:      %s", esp_err_to_name(ret));

    esp_now_register_recv_cb(ESPNow::read_message);

    add_peers();
}

// Registers every board in the MAC table as a peer, skipping this board's own address.
void ESPNow::add_peers() {
    for (int i = 0; i < PEER_COUNT; i++) {
        if (memcmp(mac_address_array[i], own_mac_address, 6) != 0) {
            esp_now_peer_info_t peer{};
            memcpy(peer.peer_addr, mac_address_array[i], 6);
            peer.channel = 0;
            peer.encrypt = false;
            peer.ifidx   = WIFI_IF_STA;
            esp_now_add_peer(&peer);
        }
    }
}

// Sends a raw byte buffer to the board identified by role. Logs the first three sends and every 100th.
void ESPNow::send_message(ROLE role, const uint8_t* data, size_t len) {
    const uint8_t* addr = nullptr;
    switch (role) {
        case PRIMARY:  addr = mac_address_array[0]; break;
        case LEADER:   addr = mac_address_array[1]; break;
        case FOLLOWER: addr = mac_address_array[2]; break;
        default: return;
    }
    static uint32_t s_send_count = 0;
    esp_err_t ret = esp_now_send(addr, data, len);
    ++s_send_count;
    if (s_send_count <= 3 || s_send_count % 100 == 0)
        ESP_LOGI(TAG, "send #%" PRIu32 " role=%d: %s",
                 s_send_count, (int)role, esp_err_to_name(ret));
}

void ESPNow::set_positions_callback(void (*cb)(ROLE, const MSGALLPOSITIONS*)) {
    g_positions_received_cb = cb;
}

void ESPNow::set_mode_callback(void (*cb)(OperationMode)) {
    g_mode_received_cb = cb;
}

// Receives every incoming ESP-NOW packet, identifies its type from the first byte,
// and forwards it to the appropriate callback.
void ESPNow::read_message(const esp_now_recv_info_t* info,
                          const uint8_t* data, int data_len) {
    if (data_len < 1) return;

    switch (static_cast<MSG_TYPE>(data[0])) {
        case MODE_OF_OPERATION: {
            if (data_len < (int)sizeof(MSGOPERATIONMODE)) return;
            if (g_mode_received_cb) {
                const MSGOPERATIONMODE* msg =
                    reinterpret_cast<const MSGOPERATIONMODE*>(data);
                g_mode_received_cb(static_cast<OperationMode>(msg->mode));
            }
            break;
        }
        case COORDINATES: {
            if (data_len < (int)sizeof(MSGCOORDINATES)) return;
            break;
        }
        case ALL_POSITIONS: {
            if (data_len < (int)sizeof(MSGALLPOSITIONS)) return;
            const MSGALLPOSITIONS* msg =
                reinterpret_cast<const MSGALLPOSITIONS*>(data);
            if (g_positions_received_cb) {
                // Identify which board sent this packet by matching the source MAC
                ROLE src = PRIMARY;
                for (int i = 0; i < (int)PEER_COUNT; i++) {
                    if (memcmp(info->src_addr, mac_address_array[i], 6) == 0) {
                        src = static_cast<ROLE>(i + 1);
                        break;
                    }
                }
                g_positions_received_cb(src, msg);
            }
            break;
        }
    }
}