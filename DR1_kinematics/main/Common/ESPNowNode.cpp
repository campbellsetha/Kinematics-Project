#include <cstdio>
#include <string>

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_system.h"
    #include "esp_log.h"
}


static uint8_t mac_addr_arr[3][6] = {
    {0x10, 0x00, 0x3b, 0xcd, 0x55, 0xc8},
    {0x10, 0x00, 0x3b, 0xcb, 0x96, 0xa8},
    {0x10, 0x00, 0x3b, 0xcb, 0xe4, 0xac},
};

void init(){
    
    esp_err_t ret = nvs_flash_init();

    esp_wifi_init(WIFI_INIT_CONFIG_DEFAULT());
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_get_mac(WIFI_IF_STA, own_mac);
    esp_now_init();
    esp_now_register_recv_cb(on_data_recv);

    addPeers();
}

static void addPeers(){
    for(int i = 0; i < 3; i++) {
        if (memcmp(mac_addr_arr[i], own_mac, 6) != 0){

            esp_now_peer_info_t peer{};
            memccpy(peer.peer_addr, mac_addr_arr[i], 6);
            peer.channel = 0;
            peer.encrypt = false;
            peer.ifidx = WIFI_IF_STA;

            esp_now_add_peer(peer)
        }
    }
}

static void send_msg(Role role, const uint8_t* data, size_t len) {
    const uint8_t* delivery_address = nullptr;
    switch(role) {

        case PRIM : delivery_address = mac_addr_arr[0]; break;
        case LEAD: delivery_address = mac_addr_arr[1]; break;
        case FLOW : delivery_address = mac_addr_arr[2]; break;
        default : "No role exist in the system.";
            return;
    }
        esp_now_send(delivery_address, data, len);
}

static uint8_t read_data(string role) {
    
    return
}





