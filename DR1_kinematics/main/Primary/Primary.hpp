// Author: Seth Campbell
// Primary.hpp
// The Primary board acts as the bridge between the host computer and the robot.
// It forwards telemetry from the Leader/Follower to the host over USB, and
// relays mode commands from the host out to both arms over ESP-NOW.

#pragma once
#include <cstdint>
#include "Common/ESPNow.hpp"

class Primary {
public:
    void init();                        // Start USB serial, ESP-NOW, and the host receive task
    void set_mode(OperationMode mode);  // Broadcast a mode change to both Leader and Follower
    void waypoint();                    // TODO: waypoint coordination between Leader and Follower

private:
    ESPNow        esp_now_;
    OperationMode current_mode_;
};