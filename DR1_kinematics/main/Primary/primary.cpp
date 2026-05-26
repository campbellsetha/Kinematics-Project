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

static ESPNow ESP_NOW_NODE;
static OperationMode CURRENT_OPERATION;

static uint32_t s_pkt_count = 0;

static void onPositionsReceived(ROLE src, const MSGALLPOSITIONS* msg) {

    static const uint8_t kSync[2] = {0xAA, 0x55};
    uint8_t src_byte = static_cast<uint8_t>(src);
    usb_serial_jtag_write_bytes(kSync,     sizeof(kSync), pdMS_TO_TICKS(10));
    usb_serial_jtag_write_bytes(&src_byte, 1,             pdMS_TO_TICKS(10));
    usb_serial_jtag_write_bytes(
        reinterpret_cast<const uint8_t*>(msg),
        sizeof(MSGALLPOSITIONS),
        pdMS_TO_TICKS(10));

    if (++s_pkt_count % 50 == 1)
        ESP_LOGI("Primary", "forwarded pkt #%" PRIu32 " src=%d J1=%u J2=%u",
                 s_pkt_count, (int)src, msg->positions[0], msg->positions[1]);
}

static void primaryRxTask(void* arg) {
    Primary* self = static_cast<Primary*>(arg);
    uint8_t buf[4];
    while (true) {
        int len = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (len > 0)
            self->setOperationMode(static_cast<OperationMode>(buf[0]));
    }
}

void Primary::init() {
    usb_serial_jtag_driver_config_t usbCfg = {};
    usbCfg.rx_buffer_size = 256;
    usbCfg.tx_buffer_size = 256;
    usb_serial_jtag_driver_install(&usbCfg);

    ESP_NOW_NODE.setPositionsCallback(onPositionsReceived);
    ESP_NOW_NODE.init();

    xTaskCreate(primaryRxTask, "primary_rx", 4096, this, 5, nullptr);
}

void Primary::setOperationMode(OperationMode operation) {
    CURRENT_OPERATION = operation;

    MSGOPERATIONMODE msg;
    msg.msgTyp = MODE_OF_OPERATION;
    msg.mode   = static_cast<uint8_t>(operation);
    ESP_NOW_NODE.sendMessage(LEADER,   reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
    ESP_NOW_NODE.sendMessage(FOLLOWER, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void Primary::waypoint() {

}
