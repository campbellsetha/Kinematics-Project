#include "ServoCommunication.hpp"
#include <cstring>
#include <vector>

extern "C" {
    #include "esp_err.h"
    #include "driver/uart.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#include <driver/gpio.h>
}

uint8_t servoIdArray[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
uint16_t initial        = 0xFFFF;

#define SERVO_UART_PORT     UART_NUM_1
#define SERVO_TX_PIN        20
#define SERVO_RX_PIN        1
#define SERVO_BAUD_RATE     1000000
#define SERVO_BUF_SIZE      256
#define SERVO_TIMEOUT_MS       10  
#define SERVO_WRITE_TIMEOUT_MS  1   

// UART setup deferred — call setupUARTCommunication() explicitly in role init()
ServoCommunication::ServoCommunication() {}

void ServoCommunication::setupUARTCommunication() {
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

// Ensures safe channel to transmit across.
void ServoCommunication::sendPacket(const uint8_t* data, int len) {
    uart_flush_input(SERVO_UART_PORT);
    uart_write_bytes(SERVO_UART_PORT, (const char*)data, len);
    uart_wait_tx_done(SERVO_UART_PORT, pdMS_TO_TICKS(10));
}

void ServoCommunication::buildCheckSum(uint8_t* msg, int len) {
    uint16_t sum = 0;
    for (int i = 2; i < len - 1; i++) // Skips the header and checksum positions
        sum += msg[i];
    msg[len - 1] = (~sum) & 0xFF;
}

bool IDCheck(uint8_t servoID) {
    return servoID >= 0x01 && servoID <= 0x06;
}

bool ServoCommunication::setTorque(int servoID, bool enable) {
    if (!IDCheck(static_cast<uint8_t>(servoID)))
        return false;

    uint8_t packet[8] = {
        0xFF,
        0xFF,
        static_cast<uint8_t>(servoID),
        0x04,
        0x03,
        0x28, // Torque applied register
        static_cast<uint8_t>(enable ? 0x01 : 0x00),
        0x00
    };
    buildCheckSum(packet, 8);

    sendPacket(packet, sizeof(packet));

    uint8_t response[6];
    int received = uart_read_bytes(SERVO_UART_PORT, response, sizeof(response),
                                   pdMS_TO_TICKS(SERVO_WRITE_TIMEOUT_MS));
    ESP_LOGI("Torque", "ID%d torque_reg=0x%02x", packet[2], packet[5]);
    return (received >= 6 && response[4] == 0x00);
}


void ServoCommunication::syncWritePositionAndSpeed(const uint16_t positions[6], const uint16_t speeds[6]) {

    static constexpr int N          = 6;
    static constexpr int DATA_LEN   = 6;   // bytes per servo (pos×2 + time×2 + spd×2)
    static constexpr int LENGTH     = 4 + N * (1 + DATA_LEN);  // 46
    static constexpr int PACKET_LEN = 3 + 1 + LENGTH;          // 50

    uint8_t pkt[PACKET_LEN] = {};
    int idx = 0;
    pkt[idx++] = 0xFF;
    pkt[idx++] = 0xFF;
    pkt[idx++] = 0xFE; // broadcast ID
    pkt[idx++] = LENGTH;
    pkt[idx++] = 0x83;
    pkt[idx++] = 0x2A; // start address
    pkt[idx++] = DATA_LEN;

    for (int i = 0; i < N; i++) {
        uint16_t spd = speeds[i] < 100 ? 100 : speeds[i];  // minimum 100 steps/sec
        pkt[idx++] = static_cast<uint8_t>(i + 1);                      // servo ID
        pkt[idx++] = static_cast<uint8_t>(positions[i] & 0xFF);        // Goal_Position L
        pkt[idx++] = static_cast<uint8_t>((positions[i] >> 8) & 0xFF); // Goal_Position H
        pkt[idx++] = 0x00;                                              // Goal_Time L = 0
        pkt[idx++] = 0x00;                                              // Goal_Time H = 0
        pkt[idx++] = static_cast<uint8_t>(spd & 0xFF);                 // Running_Speed L
        pkt[idx++] = static_cast<uint8_t>((spd >> 8) & 0xFF);          // Running_Speed H
    }

    buildCheckSum(pkt, PACKET_LEN);

    ESP_LOG_BUFFER_HEX("TX_PKT", pkt, PACKET_LEN);
    
    sendPacket(pkt, PACKET_LEN);
}

std::vector<uint16_t> ServoCommunication::readAllServoPositions() {
    std::vector<uint16_t> servoPositions;

    for (int i = 0; i < 6; i++) {
        uint8_t packet[8] = {
            0xFF,
            0xFF,
            static_cast<uint8_t>(i + 1),
            0x04,
            0x02,
            0x38,
            0x02,
            0x00
        };
        buildCheckSum(packet, 8);

        sendPacket(packet, sizeof(packet));

        uint8_t response[8];
        int received = uart_read_bytes(SERVO_UART_PORT, response, sizeof(response),
                                       pdMS_TO_TICKS(10));

        ESP_LOGI("Servo", "ID%d rx=%d [%02x %02x %02x %02x %02x %02x %02x %02x]",
                 i+1, received,
                 response[0], response[1], response[2], response[3],
                 response[4], response[5], response[6], response[7]);

        if (received < 8 || response[4] != 0x00) {
            servoPositions.push_back(0xFFFF);
            continue;
        }

        uint16_t pos = (static_cast<uint16_t>(response[6]) << 8) | response[5];
        servoPositions.push_back(pos);
    }

    return servoPositions;
}
