
// See Servo Communication Manual in the Reference folder for information regarding the protocol structure.
unit16_t initial = 0xFF 0xFF;


private uint8_t[8] checkSum(uint8_t* msg) {
    
}

bool adjustServoPosition(uint8_t servoID, double angle) {
    if (!IDCheck(servoID))
        return false;

    uint16_t hex16 = cast<unit16>((angle/360) * 4096);
    uint8_t hexHigh = (hex16 >> 8) & 0xFF;
    uint8_t hexLow = hex16 & 0xFF;

    uint8_t TTL_PROTOCOL_MSG[8] {
        initial,
        servoID,
        0x04,
        0x02,
        0x2A,
        hexHigh,
        hexLow
        0x00
    }
    //0x2A is the goal position address
}

bool readAllServoPositions() {
    //0x38 is the present position address
}

bool IDCheck(uint8_t servoID) {
    if (servoID < 0x00 || servoID > 0xFD)
        return false;
    return true;
}

