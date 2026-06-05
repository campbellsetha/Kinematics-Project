// Author: Seth Campbell
// ESPNow.hpp
// Defines the message types and ESPNow class used for wireless communication
// between the Leader, Follower, and Primary (host) boards over ESP-NOW.

#pragma once
#pragma pack(push, 1)

#include <cstdint>

extern "C" {
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_err.h"
}

extern uint8_t mac_address_array[3][6]; // MAC addresses for all three boards

// Which board a message is addressed to or came from
enum ROLE {
    PRIMARY  = 1,
    LEADER   = 2,
    FOLLOWER = 3
};

// High-level operating modes sent from the host to control robot behavior
enum OperationMode : uint8_t {
    TELEOP    = 0x00, // Mirror leader arm movements live on the follower
    WAYPNT    = 0x01, // Execute a saved waypoint sequence
    AXIS_TEST = 0x02  // Move one joint at a time for testing
};

// First byte of every packet — identifies which struct follows
enum MSG_TYPE : uint8_t {
    MODE_OF_OPERATION = 0x00,
    COORDINATES       = 0x10,
    ALL_POSITIONS     = 0xFF
};

// Main telemetry packet: 6 servo positions plus end-effector XYZ in metres (25 bytes total)
struct MSGALLPOSITIONS {
    uint8_t  msg_type;
    uint16_t positions[6]; // Raw servo tick values (0-4095)
    float    ee_x;
    float    ee_y;
    float    ee_z;
};

// Tells all boards to switch operating mode
struct MSGOPERATIONMODE {
    uint8_t msg_type;
    uint8_t mode;
};

// Commands a single servo to a specific position
struct MSGSINGLESERVO {
    uint8_t  msg_type;
    uint8_t  servo_id;
    uint16_t position;
};

// Sends a target XYZ coordinate (used for IK navigation)
struct MSGCOORDINATES {
    uint8_t msg_type;
    float x, y, z;
};

#pragma pack(pop)

extern uint8_t own_mac_address[6]; // This board's own MAC, populated at init

class ESPNow {
public:
    ESPNow() = default;

    void init();                                                            // Start WiFi and ESP-NOW, register all peers
    void send_message(ROLE role, const uint8_t* data, size_t len);         // Send a raw packet to a specific board
    void set_positions_callback(void (*cb)(ROLE, const MSGALLPOSITIONS*)); // Register handler for incoming servo position data
    void set_mode_callback(void (*cb)(OperationMode));                     // Register handler for incoming mode change commands

private:
    void add_peers();                                                       // Register all boards as ESP-NOW peers at startup
    static void read_message(const esp_now_recv_info_t* info,
                             const uint8_t* data, int data_len);           // Dispatch incoming packets to the right callback

    static constexpr uint8_t PEER_COUNT = 3;
};