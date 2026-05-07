#pragma once

#include <cstdint>

uint8_t servoIdArray[6]={0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
uint16_t initial = 0xFFFF;

class ServoCommunication {
    public:
        ServoCommunication() = default;
        adjustServoPosition(int servoID, double angle)
        
    private:
        uint8_t[8] checkSum(uint8_t* msg);
};
