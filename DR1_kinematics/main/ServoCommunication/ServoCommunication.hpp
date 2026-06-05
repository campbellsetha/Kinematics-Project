// Author: Seth Campbell
// ServoCommunication.hpp
// Declares the ServoCommunication class that handles all low-level UART
// communication with the servo bus — sending commands and reading positions.

#pragma once

#include <cstdint>
#include <vector>

extern uint8_t  servoIdArray[6]; // Hardware IDs for all six servos (1–6)
extern uint16_t initial;         // Sentinel value (0xFFFF) used to flag invalid/unread positions

class ServoCommunication {
    public:
        ServoCommunication();
        void setup_uart();                                  // Configure and start the UART bus at 1 Mbaud
        void set_torque(int servoID, bool enable);          // Enable or disable holding torque on a servo
        void write_servo_position(int servoID, uint16_t position); // Command a servo to move to a target tick position
        std::vector<uint16_t> read_servo_positions();       // Poll all 6 servos and return their current positions (0xFFFF on failure)

    private:
        void send_packet(const uint8_t* data, int len);    // Flush a raw byte packet out over UART
        void build_check_sum(uint8_t* msg, int len);       // Compute and write the Dynamixel checksum into the last byte of a packet
};