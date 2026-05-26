#pragma once

#include <cstdint>
#include <vector>

extern uint8_t servoIdArray[6];
extern uint16_t initial;

bool IDCheck(uint8_t servoID);

class ServoCommunication {
    public:
        ServoCommunication();
        void setupUARTCommunication();
        bool adjustServoPosition(int servoID, uint16_t angle);
        bool adjustServoSpeed(int servoID, uint16_t speed);
        bool setTorque(int servoID, bool enable);
        void syncWritePositionAndSpeed(const uint16_t positions[6], const uint16_t speeds[6]);
        std::vector<uint16_t> readAllServoPositions();
        std::vector<uint16_t> readAllServoSpeeds();

    private:
        void sendPacket(const uint8_t* data, int len);
        void buildCheckSum(uint8_t* msg, int len);
};
