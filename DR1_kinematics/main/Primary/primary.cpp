// Author: Seth Campbell
// Primary.cpp
// Bridges the host computer and the robot over USB serial + ESP-NOW.
// Incoming position packets from Leader/Follower are forwarded to the host;
// mode bytes from the host are broadcast to both arms.

#include <cstdio>
#include <cinttypes>
#include "Primary.hpp"

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_log.h"
    #include "driver/usb_serial_jtag.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

static uint32_t s_pkt_count = 0;

// Receives position packets from either arm and forwards them to the host.
// Wire format sent over USB: [0xAA][0x55][src_byte][MSGALLPOSITIONS — 25 bytes] = 28 bytes total.
static void on_positions_received(ROLE src, const MSGALLPOSITIONS* msg) {
    static const uint8_t k_sync[2] = {0xAA, 0x55};
    uint8_t src_byte = static_cast<uint8_t>(src);

    usb_serial_jtag_write_bytes(k_sync,    sizeof(k_sync),        pdMS_TO_TICKS(10));
    usb_serial_jtag_write_bytes(&src_byte, 1,                     pdMS_TO_TICKS(10));
    usb_serial_jtag_write_bytes(
        reinterpret_cast<const uint8_t*>(msg),
        sizeof(MSGALLPOSITIONS),
        pdMS_TO_TICKS(10));

    // Log every 50th packet to confirm data is flowing without flooding the console
    if (++s_pkt_count % 50 == 1)
        ESP_LOGI("Primary", "forwarded pkt #%" PRIu32 " src=%d J1=%u J2=%u",
                 s_pkt_count, (int)src, msg->positions[0], msg->positions[1]);
}

// FreeRTOS task: watches for mode bytes from the host and applies them via set_mode().
static void primary_rx_task(void* arg) {
    Primary* self = static_cast<Primary*>(arg);
    uint8_t buf[4];
    while (true) {
        int len = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (len > 0)
            self->set_mode(static_cast<OperationMode>(buf[0]));
    }
}

// Initialises USB serial, ESP-NOW, registers the position callback, and starts the host receive task.
void Primary::init() {
    usb_serial_jtag_driver_config_t usb_cfg = {};
    usb_cfg.rx_buffer_size = 256;
    usb_cfg.tx_buffer_size = 256;
    usb_serial_jtag_driver_install(&usb_cfg);

    esp_now_.set_positions_callback(on_positions_received);
    esp_now_.init();

    xTaskCreate(primary_rx_task, "primary_rx_task", 4096, this, 5, nullptr);
}

// Stores the new mode locally and broadcasts a MODE_OF_OPERATION packet to Leader and Follower.
void Primary::set_mode(OperationMode mode) {
    current_mode_ = mode;

    MSGOPERATIONMODE msg;
    msg.msg_type = MODE_OF_OPERATION;
    msg.mode     = static_cast<uint8_t>(mode);
    esp_now_.send_message(LEADER,
        reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
    esp_now_.send_message(FOLLOWER,
        reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void Primary::waypoint() {
    // TODO: waypoint coordination between Leader and Follower
}