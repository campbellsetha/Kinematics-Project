// Author: Seth Campbell
// Leader.hpp
// The Leader reads its own servo positions, runs forward kinematics to find
// the end-effector position, and broadcasts both to the Follower and Primary.

#pragma once
#include <cstdint>
#include "Common/ESPNow.hpp"
#include "ServoCommunication/ServoCommunication.hpp"
#include "Common/Frame.hpp"
#include "Forward_K.hpp"

class Leader {
public:
    Leader();
    void init();                    // Set up servos, ESP-NOW, FK chain, and launch the transmit task
    void set_mode(OperationMode mode);
    void read_and_transmit();       // Read servo ticks, compute FK, and broadcast the position packet
    void waypoint();                // TODO: capture and transmit waypoints to the Follower

private:
    ServoCommunication servo_bus_;  // Handles UART communication with the physical servos
    ESPNow             esp_now_;    // Handles wireless communication with the Follower and Primary
    OperationMode      current_mode_;
    Frame              frame_;      // The robot's joint chain used for FK
    ForwardK           fk_;        // Computes end-effector XYZ from live servo readings
};