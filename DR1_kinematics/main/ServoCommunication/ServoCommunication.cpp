// Author: Seth Campbell
// ServoCommunication.cpp
// Implements UART-based communication with the servo bus using the Dynamixel protocol.
// Builds, checksums, and sends command packets; reads and parses position response packets.

#include "ServoCommunication.hpp"
#include <cstring>
#include <vector>

extern "C" {
    #include "esp_err.h"
    #include "driver/uart.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include <driver/gpio.h>
    #include <rom/ets_sys.h>
}

uint8_t  servoIdArray[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
uint16_t initial         = 0xFFFF; // Returned when a servo read fails or times out

// UART hardware configuration
#define SERVO_UART_PORT         UART_NUM_1
#define SERVO_TX_PIN            20
#define SERVO_RX_PIN            1
#define SERVO_BAUD_RATE         1000000
#define SERVO_BUF_SIZE          256
#define SERVO_TIMEOUT_MS        10
#define SERVO_WRITE_TIMEOUT_MS  1

ServoCommunication::ServoCommunication() {}

// Configures UART1 at 1 Mbaud 8N1, installs the driver, and pulls RX high for open-drain bus compatibility.
void ServoCommunication::setup_uart() {
    uart_config_t CONFIG = {};
    CONFIG.baud_rate = SERVO_BAUD_RATE;
    CONFIG.data_bits = UART_DATA_8_BITS;
    CONFIG.parity    = UART_PARITY_DISABLE;
    CONFIG.stop_bits = UART_STOP_BITS_1;
    CONFIG.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    uart_param_config(SERVO_UART_PORT, &CONFIG);
    uart_set_pin(SERVO_UART_PORT, SERVO_TX_PIN, SERVO_RX_PIN,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(SERVO_UART_PORT, SERVO_BUF_SIZE * 2, 0, 0, NULL, 0);
    gpio_set_pull_mode((gpio_num_t)SERVO_RX_PIN, GPIO_PULLUP_ONLY);
    uart_flush_input(SERVO_UART_PORT);
}

// Waits 2 ms for the bus to settle, writes the packet, then blocks until TX is complete.
void ServoCommunication::send_packet(const uint8_t* data, int len) {
    ets_delay_us(2000);
    uart_write_bytes(SERVO_UART_PORT, (const char*)data, len);
    uart_wait_tx_done(SERVO_UART_PORT, pdMS_TO_TICKS(10));
}

// Sums bytes [2..len-2], inverts the result, and stores it in the last byte — standard Dynamixel checksum.
void ServoCommunication::build_check_sum(uint8_t* msg, int len) {
    uint16_t sum = 0;
    for (int i = 2; i < len - 1; i++)
        sum += msg[i];
    msg[len - 1] = (~sum) & 0xFF;
}

// Sends a torque enable/disable command to a single servo (register 0x28).
void ServoCommunication::set_torque(int servoID, bool enable) {
    uint8_t packet[8] = {
        0xFF,
        0xFF,
        static_cast<uint8_t>(servoID),
        0x04,   // LENGTH
        0x03,   // WRITE DATA instruction
        0x28,   // Torque_Enable register
        static_cast<uint8_t>(enable ? 0x01 : 0x00),
        0x00    // checksum slot
    };

    build_check_sum(packet, 8);
    send_packet(packet, sizeof(packet));
}

// Writes Goal_Position (register 0x2A) to a single servo.
// Running_Time is set to 0 so the servo uses its pre-configured speed register.
// Positions are written one servo at a time (not sync-write) to avoid simultaneous
// inrush current across all motors on the shared power rail.
void ServoCommunication::write_servo_position(int servoID, uint16_t position) {
    uint8_t packet[11] = {
        0xFF,
        0xFF,
        static_cast<uint8_t>(servoID),
        0x07,                                          // LENGTH
        0x03,                                          // WRITE DATA instruction
        0x2A,                                          // Goal_Position L register
        static_cast<uint8_t>(position & 0xFF),         // pos low byte
        static_cast<uint8_t>((position >> 8) & 0xFF),  // pos high byte
        0x00,                                          // Running_Time L = 0 (use speed register)
        0x00,                                          // Running_Time H = 0
        0x00                                           // checksum slot
    };

    build_check_sum(packet, sizeof(packet));
    send_packet(packet, sizeof(packet));
}

// Polls all 6 servos sequentially for their current position (register 0x38, 2 bytes).
// Returns a vector of 6 tick values; 0xFFFF means that servo did not respond or returned an error.
std::vector<uint16_t> ServoCommunication::read_servo_positions() {
    std::vector<uint16_t> servoPositions;

    uart_flush_input(SERVO_UART_PORT);
    ets_delay_us(2000);

    for (int i = 0; i < 6; i++) {
        uint8_t packet[8] = {
            0xFF,
            0xFF,
            static_cast<uint8_t>(i + 1),
            0x04,   // LENGTH
            0x02,   // READ DATA instruction
            0x38,   // Present_Position L register
            0x02,   // read 2 bytes
            0x00    // checksum slot
        };
        build_check_sum(packet, 8);
        send_packet(packet, sizeof(packet));

        uint8_t response[8] = {};
        int received = uart_read_bytes(SERVO_UART_PORT, response, sizeof(response),
                                       pdMS_TO_TICKS(10));

        if (received < 8 || response[4] != 0x00) {
            servoPositions.push_back(0xFFFF); // No response or error flag set
            continue;
        }

        // Position is little-endian across bytes 5 and 6 of the response
        uint16_t pos = (static_cast<uint16_t>(response[6]) << 8) | response[5];
        servoPositions.push_back(pos);
    }

    return servoPositions;
}