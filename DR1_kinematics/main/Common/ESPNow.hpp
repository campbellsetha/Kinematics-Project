#pragma once

#include <cstdint>

extern "C" {
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_err.h"
}

static uint8_t MACAddressArray[3][6] = {
    {0x10, 0x00, 0x3b, 0xcd, 0x55, 0xc8},
    {0x10, 0x00, 0x3b, 0xcb, 0x96, 0xa8},
    {0x10, 0x00, 0x3b, 0xcb, 0xe4, 0xac},
};

struct jointStatus {
    int j0;
    uint16_t j0Position;

    int j1;
    uint16_t j1Position;

    int j2;
    uint16_t j2Position;

    int j3;
    uint16_t j3Position;

    int j4;
    uint16_t j4Position;

    int j5;
    uint16_t j5Position;
};

uint8_t ownMacAddress[6];

class ESPNow {
public:
    ESPNow() = default;

    void init();
    void sendMessage(int role, const uint8_t* data, size_t len);
    uint8_t* readMessage(esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

private:
    void addPeers();

    static constexpr uint8_t PEER_COUNT = 3;
    static uint8_t mac_addr_arr[PEER_COUNT][6];

    uint8_t own_mac[6];
};