#include <vector>
#include "Follower.hpp"
#include "Common/ESPNow.hpp"
#include "ServoCommunication/ServoCommunication.hpp"

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
}

static ESPNow ESP_NOW_NODE;
static ServoCommunication SERVO_BUS_NODE;
static OperationMode CURRENT_OPERATION;

// Depth-1 queue decouples the ESP-NOW WiFi callback from blocking UART servo writes.
static QueueHandle_t s_positions_queue = nullptr;

static void onPositionsReceived(ROLE /*src*/, const MSGALLPOSITIONS* msg) {
    // Called from the ESP-NOW/WiFi task — must not block. Post latest frame and return.
    xQueueOverwrite(s_positions_queue, msg);
}

static void servoTask(void*) {
    MSGALLPOSITIONS msg;
    uint16_t last_positions[6] = {2048, 2048, 2048, 2048, 2048, 2048};
    uint16_t last_speeds[6]    = {200,  200,  200,  200,  200,  200};

    // Seed last_positions from actual servo positions so the
    // first received leader frame doesn't snap the arm from 2048.
    {
        std::vector<uint16_t> boot_pos = SERVO_BUS_NODE.readAllServoPositions();
        if (boot_pos.size() == 6)
            for (int i = 0; i < 6; i++)
                if (boot_pos[i] != 0xFFFF) last_positions[i] = boot_pos[i];
    }

    static constexpr uint16_t DEADBAND    = 4;
    static constexpr uint16_t MIN_SPEED   = 200;
    static constexpr uint16_t MAX_SPEED   = 1500;
    static constexpr uint16_t SPEED_SCALE = 50;

    while (true) {
        if (xQueueReceive(s_positions_queue, &msg, portMAX_DELAY) != pdTRUE) continue;

        ESP_LOGI("FOLLOWER", "RX [%d %d %d %d %d %d]",
        msg.positions[0], msg.positions[1], msg.positions[2],
        msg.positions[3], msg.positions[4], msg.positions[5]);

        for (int i = 0; i < 6; i++) {
            if (msg.positions[i] == 0xFFFF) continue;
            int delta = abs((int)msg.positions[i] - (int)last_positions[i]);
            if (delta < DEADBAND) continue;
            last_positions[i] = msg.positions[i];
            uint16_t spd = (uint16_t)(delta * SPEED_SCALE);
            last_speeds[i] = spd < MIN_SPEED ? MIN_SPEED
                           : spd > MAX_SPEED ? MAX_SPEED : spd;
        }

        ESP_LOGI("FOLLOWER", "WRITE [%d %d %d %d %d %d]",
        last_positions[0], last_positions[1], last_positions[2],
        last_positions[3], last_positions[4], last_positions[5]);

        SERVO_BUS_NODE.syncWritePositionAndSpeed(last_positions, last_speeds);
        vTaskDelay(pdMS_TO_TICKS(5));

        std::vector<uint16_t> actual = SERVO_BUS_NODE.readAllServoPositions();
        if (actual.size() == 6) {
            bool any_valid = false;
            for (int i = 0; i < 6; i++) if (actual[i] != 0xFFFF) { any_valid = true; break; }
            if (any_valid) {
                MSGALLPOSITIONS feedback;
                feedback.msgTyp = ALL_POSITIONS;
                for (int i = 0; i < 6; i++) feedback.positions[i] = actual[i];
                for (int i = 0; i < 6; i++) feedback.speeds[i]    = 0xFFFF;
                ESP_NOW_NODE.sendMessage(PRIMARY,
                    reinterpret_cast<const uint8_t*>(&feedback), sizeof(feedback));
            }
        }
    }
}

void Follower::init() {
    s_positions_queue = xQueueCreate(1, sizeof(MSGALLPOSITIONS));
    SERVO_BUS_NODE.setupUARTCommunication();

    vTaskDelay(pdMS_TO_TICKS(500));       // let servos finish power-up
    for (int i = 1; i < 7; i++)
        SERVO_BUS_NODE.setTorque(i, true);

    ESP_NOW_NODE.setPositionsCallback(onPositionsReceived);
    ESP_NOW_NODE.init();
    xTaskCreate(servoTask, "servo", 4096, nullptr, 5, nullptr);
}

void Follower::setOperationMode(OperationMode operation) {
    CURRENT_OPERATION = operation;
}

void Follower::applyPositions(const MSGALLPOSITIONS* msg) {
    for (int i = 0; i < 6; i++) {
        if (msg->positions[i] == 0xFFFF || msg->speeds[i] == 0xFFFF) continue;
        SERVO_BUS_NODE.adjustServoSpeed(i + 1, msg->speeds[i]);
        SERVO_BUS_NODE.adjustServoPosition(i + 1, msg->positions[i]);
    }
}

void Follower::waypoint() {

}
