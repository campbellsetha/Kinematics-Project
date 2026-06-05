// Author: Seth Campbell
// Follower.cpp
// Receives servo positions from the Leader and drives the follower arm to match.
// Uses a depth-1 queue so only the most recent frame is ever acted on.

#include <vector>
#include "Follower.hpp"

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include <rom/ets_sys.h>
}

// Depth-1 queue — xQueueOverwrite ensures only the latest leader frame is kept
static QueueHandle_t s_positions_queue = nullptr;
static Follower*     s_follower_instance = nullptr;

// Callback: drops incoming position frames onto the queue (only accepts packets from the Leader)
static void on_positions_received(ROLE src, const MSGALLPOSITIONS* msg) {
    if (src != LEADER) return;
    xQueueOverwrite(s_positions_queue, msg);
}

// Callback: forwards mode changes to the Follower instance
static void on_mode_received(OperationMode mode) {
    if (s_follower_instance) s_follower_instance->set_mode(mode);
}

static void follower_task(void* arg) {
    static_cast<Follower*>(arg)->servo_loop();
}

// Initialises hardware and networking, enables torque on all servos, then launches the servo task.
void Follower::init() {
    s_follower_instance  = this;
    s_positions_queue    = xQueueCreate(1, sizeof(MSGALLPOSITIONS));

    servo_bus_.setup_uart();

    esp_now_.set_positions_callback(on_positions_received);
    esp_now_.set_mode_callback(on_mode_received);
    esp_now_.init();

    for (int i = 1; i <= 6; i++) {
        servo_bus_.set_torque(i, true);
    }
    
    xTaskCreate(follower_task, "follower_task", 4096, this, 5, nullptr);
}

void Follower::set_mode(OperationMode mode) {
    current_mode_ = mode;
}

// Main servo loop: waits for a position frame from the Leader, computes per-joint speed
// proportional to how far each servo needs to move, then writes positions staggered in time
// to avoid voltage spikes from simultaneous motor inrush. Sends actual positions back to Primary.
void Follower::servo_loop() {
    MSGALLPOSITIONS msg;

    uint16_t positions[6] = {2048, 2048, 2048, 2048, 2048, 2048};
    uint16_t speeds[6]    = {200,  200,  200,  200,  200,  200};

    // Seed starting positions from actual servo readings rather than assuming centre
    {
        std::vector<uint16_t> boot_pos = servo_bus_.read_servo_positions();
        if (boot_pos.size() == 6)
            for (int i = 0; i < 6; i++)
                if (boot_pos[i] != 0xFFFF) positions[i] = boot_pos[i];
    }

    // Movement tuning — adjust these to change responsiveness and smoothness
    static constexpr uint16_t DEADBAND          = 2;    // Ignore movements smaller than this (ticks)
    static constexpr uint16_t MIN_SPEED         = 200;  // Slowest allowed servo speed
    static constexpr uint16_t MAX_SPEED         = 1500; // Fastest allowed servo speed
    static constexpr uint16_t SPEED_SCALE       = 50;   // Multiplier: speed = delta * SPEED_SCALE

    while (true) {
        if (xQueueReceive(s_positions_queue, &msg, portMAX_DELAY) != pdTRUE) continue;

        if (current_mode_ != TELEOP) continue;

        ESP_LOGI("FOLLOWER", "RX  [%d %d %d %d %d %d]",
                 msg.positions[0], msg.positions[1], msg.positions[2],
                 msg.positions[3], msg.positions[4], msg.positions[5]);

        for (int i = 0; i < 6; i++) {
            if (msg.positions[i] == 0xFFFF) continue;
            int delta = abs((int)msg.positions[i] - (int)positions[i]);
            if (delta < DEADBAND) {
                speeds[i] = MIN_SPEED;
                continue;
            }
            positions[i] = msg.positions[i];
            uint16_t spd = static_cast<uint16_t>(delta * SPEED_SCALE);
            speeds[i] = spd < MIN_SPEED ? MIN_SPEED
                      : spd > MAX_SPEED ? MAX_SPEED : spd;
        }

        ESP_LOGI("FOLLOWER", "WR  pos=[%d %d %d %d %d %d] spd=[%d %d %d %d %d %d]",
                 positions[0], positions[1], positions[2],
                 positions[3], positions[4], positions[5],
                 speeds[0],    speeds[1],    speeds[2],
                 speeds[3],    speeds[4],    speeds[5]);

        // Write each servo sequentially with a short delay between them to prevent
        // simultaneous inrush current from causing a voltage sag on the shared power rail.
        static constexpr TickType_t INTER_SERVO_DELAY_MS = 50;

        for (int i = 0; i < 6; i++) {
            servo_bus_.write_servo_position(i + 1, positions[i]);
            vTaskDelay(pdMS_TO_TICKS(INTER_SERVO_DELAY_MS));
        }

        // Read back actual positions and report to Primary for telemetry
        std::vector<uint16_t> actual = servo_bus_.read_servo_positions();
        if (actual.size() == 6) {
            bool any_valid = false;
            for (int i = 0; i < 6; i++)
                if (actual[i] != 0xFFFF) { any_valid = true; break; }

            if (any_valid) {
                MSGALLPOSITIONS feedback;
                feedback.msg_type = ALL_POSITIONS;
                for (int i = 0; i < 6; i++) feedback.positions[i] = actual[i];
                feedback.ee_x = 0.0f;
                feedback.ee_y = 0.0f;
                feedback.ee_z = 0.0f;
                esp_now_.send_message(PRIMARY,
                    reinterpret_cast<const uint8_t*>(&feedback), sizeof(feedback));
            }
        }
    }
}

void Follower::waypoint() {
    // TODO: receive waypoint packet, go to home, execute IK sequence with 2s hold, return home
}