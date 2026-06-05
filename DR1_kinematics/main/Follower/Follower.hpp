// Author: Seth Campbell
// Follower.hpp
// The Follower receives servo positions from the Leader over ESP-NOW and
// drives its own servos to match, mirroring the Leader arm in real time.

#pragma once
#include <cstdint>
#include "Common/ESPNow.hpp"
#include "ServoCommunication/ServoCommunication.hpp"

class Follower {
    ServoCommunication servo_bus_;  // Handles UART communication with the physical servos
    ESPNow             esp_now_;    // Handles wireless communication with the Leader and Primary
    OperationMode      current_mode_;
public:
    void init();                    // Start ESP-NOW, enable servos, and launch the servo task
    void set_mode(OperationMode mode);
    void servo_loop();              // Main task loop — receives positions and drives servos (do not call directly)
    void waypoint();                // TODO: execute a saved waypoint sequence
};