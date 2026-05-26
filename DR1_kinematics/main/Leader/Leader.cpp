#include <cstdio>
#include <vector>
#include "Leader.hpp"
#include "ServoCommunication/ServoCommunication.hpp"

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

static ESPNow ESP_NOW_NODE;
static ServoCommunication SERVO_BUS_NODE;
static OperationMode CURRENT_OPERATION;

static void leaderTask(void* arg) {
    Leader* self = static_cast<Leader*>(arg);
    while (true) {
        self->getPositions();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void Leader::init() {
    SERVO_BUS_NODE.setupUARTCommunication();
    for (int i = 1; i <= 6; i++)
        SERVO_BUS_NODE.setTorque(i, false);
    
    ESP_NOW_NODE.init();
    xTaskCreate(leaderTask, "leader", 4096, this, 5, nullptr);
}

void Leader::setOperationMode(OperationMode operation) {
    CURRENT_OPERATION = operation;
}

void Leader::getPositions() {
    std::vector<uint16_t> positions = SERVO_BUS_NODE.readAllServoPositions();
    if (positions.size() < 6) return;

    MSGALLPOSITIONS msg;
    msg.msgTyp = ALL_POSITIONS;
    for (int i = 0; i < 6; i++) {
        msg.positions[i] = positions[i];
        msg.speeds[i]    = 0xFFFF; // unused — follower computes speed from delta
    }

    ESP_NOW_NODE.sendMessage(PRIMARY,
        reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));

    if (CURRENT_OPERATION == TELEOP)
        ESP_NOW_NODE.sendMessage(FOLLOWER,
            reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void Leader::waypoint() {

}
