// Author: Seth Campbell
// Leader.cpp
// Reads the leader arm's servo positions every 20 ms, computes forward kinematics,
// and broadcasts the position packet to Primary and the Follower (in TELEOP).

#include <cstdio>
#include <vector>
#include "Leader.hpp"

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

static Leader* s_leader_instance = nullptr;

// Callback: forwards mode changes from the Primary to the Leader instance
static void on_mode_received(OperationMode mode) {
    if (s_leader_instance) s_leader_instance->set_mode(mode);
}

// FreeRTOS task: calls read_and_transmit() on the Leader every 20 ms
static void leader_task(void* arg) {
    Leader* self = static_cast<Leader*>(arg);
    while (true) {
        self->read_and_transmit();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

Leader::Leader()
    : current_mode_(TELEOP),
      fk_(frame_, JOINT_OFFSETS.data()) {}

// Initialises the joint chain, UART, ESP-NOW, disables servo torque (leader is back-drivable),
// and launches the transmit task with an 8 KB stack to handle FK matrix work.
void Leader::init() {
    s_leader_instance = this;
    init_chain(frame_);
    servo_bus_.setup_uart();
    for (int i = 1; i <= 6; i++)
        servo_bus_.set_torque(i, false);
    esp_now_.set_mode_callback(on_mode_received);
    esp_now_.init();
    xTaskCreate(leader_task, "leader_task", 8192, this, 5, nullptr);
}

void Leader::set_mode(OperationMode mode) {
    current_mode_ = mode;
}

// Reads current servo ticks, runs FK to get end-effector XYZ, and sends the combined
// packet to Primary always and to Follower only when in TELEOP mode.
void Leader::read_and_transmit() {
    std::vector<uint16_t> raw = servo_bus_.read_servo_positions();
    if (raw.size() < 6) return;

    uint16_t ticks[6];
    for (int i = 0; i < 6; i++)
        ticks[i] = raw[i];

    bool invalid = false;
    for (int i = 0; i < 6; i++)
        if (ticks[i] == 0xFFFF) {
            invalid = true; 
            break;
        }
    
    MSGALLPOSITIONS msg;  
    msg.msg_type = ALL_POSITIONS;

    if (!invalid) {
        auto ee = fk_.compute(ticks);
        msg.ee_x = static_cast<float>(ee[0]);
        msg.ee_y = static_cast<float>(ee[1]);
        msg.ee_z = static_cast<float>(ee[2]);
        ESP_LOGI("FK", "EE x=%.4f y=%.4f z=%.4f (m)", ee[0], ee[1], ee[2]);
    }    

    for (int i = 0; i < 6; i++)
        msg.positions[i] = ticks[i];

    esp_now_.send_message(PRIMARY,
        reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));

    if (current_mode_ == TELEOP)
        esp_now_.send_message(FOLLOWER,
            reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void Leader::waypoint() {
    // TODO: spacebar position capture, enter to confirm, transmit to Follower
}