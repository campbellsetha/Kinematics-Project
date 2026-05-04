#pragma once

#include <cstdint>

extern "C" {
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_err.h"
}

enum class Role {
    PRIM = 0,
    LEAD = 1,
    FLOW = 2
};

uint8_t own_mac[6];

class ESPNow {
public:
    ESPNow() = default;

    void init();
    void send_msg(Role role, const uint8_t* data, size_t len);
    uint8_t* read_data(Role role);

private:
    void addPeers();

    static constexpr uint8_t PEER_COUNT = 3;
    static uint8_t mac_addr_arr[PEER_COUNT][6];

    uint8_t own_mac[6];
};