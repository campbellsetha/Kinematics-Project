#include <cstdio>
#include <string>

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "ESPNow.hpp"
}


void init(){
    
    esp_err_t ret = nvs_flash_init();

    esp_wifi_init(WIFI_INIT_CONFIG_DEFAULT());
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_get_mac(WIFI_IF_STA, ownMacAddress);
    esp_now_init();
    esp_now_register_recv_cb(readMessage());

    addPeers();
}

static void addPeers(){
    for(int i = 0; i < 3; i++) {
        if (memcmp(MACAddressArray[i], ownMacAddress, 6) != 0){

            esp_now_peer_info_t peer{};
            memccpy(peer.peer_addr, MACAddressArray[i], 6);
            peer.channel = 0;
            peer.encrypt = false;
            peer.ifidx = WIFI_IF_STA;

            esp_now_add_peer(peer*);
        }
    }
}

static void sendMessage(int role, const uint8_t* data, size_t len) {
    const uint8_t* delivery_address = nullptr;
    switch(role) {

        case 0 : delivery_address = MACAddressArray[0]; break;
        case 1: delivery_address = MACAddressArray[1]; break;
        case 2 : delivery_address = MACAddressArray[2]; break;
        default : "No role exist in the system.";
            return;
    }
        esp_now_send(delivery_address, data, len);
}

static uint8_t readMessage(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (ownMacAddress == MACAddressArray[0])
        Primary.setServoPositions(data);
    if (ownMacAddress == MACAddressArray[1])
        Leader
    return;
}





