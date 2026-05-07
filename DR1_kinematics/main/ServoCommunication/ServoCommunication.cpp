#include "ServoCommunication.hpp"
#include <cstring>

extern "C" {
    #include "esp_err.h"
    #include "driver/uart.h"
}

ServoCommunication() {
    //set to pins to be used from ESP32 (should be 20(Tx), 21(Rx), but port is unknown)
}

void setupUARTCommunication() {
    uart_config_t CONFIG = {
        .baud_rate = 1000000,
        .data_bits = uart_data_,
        .parity = uart_set_parity(false),
        .stop_bits = uart_get_stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
}

private uint8_t[8] checkSum(uint8_t* msg) {
    uint8_t sum;
    for (int i = 0; i < 7; i++) {
        sum += msg[i];
    }
    if (sum > 0xFF)
        sum = sum & 0xFF;
    msg[7] = sum;
    return msg;
}

bool ServoCommunication::adjustServoPosition(int servoID, double angle) {
    if (!IDCheck(servoID))
        return false;

    uint16_t hex16 = cast<unit16>((angle/360) * 4096);
    uint8_t hexHigh = (hex16 >> 8) & 0xFF;
    uint8_t hexLow = hex16 & 0xFF;

    uint8_t TTL_PROTOCOL_MSG[8] {
        initial,
        cast<uint8_t>(servoID),
        0x04,
        0x03,
        0x2A,
        hexHigh,
        hexLow
        0x00
    }
    // Send over UART
    checkSum(TTL_PROTOCOL_MSG);

    //if reply packet 

    //0x2A is the goal position address
}

std::vector<double> ServoCommunication::readAllServoPositions() {

    std::vector<double> servoPositions;
    for (const auto& [joint : ID] : jointToServoMap){

    }

    return servoPositions;

    //0x38 is the present position address
}

bool IDCheck(uint8_t servoID) {
    if (servoID < 0x00 || servoID > 0x06)
        return false;
    return true;
}

