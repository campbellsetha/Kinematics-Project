#pragma once
#pragma pack(push, 1)

#include <cstdint>

extern "C" {
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_err.h"
}

extern uint8_t MACAddressArray[3][6];

enum ROLE {
    PRIMARY  = 1,
    LEADER   = 2,
    FOLLOWER = 3
};

enum OperationMode : uint8_t {
    TELEOP = 0x00,
    WAYPNT = 0x01
};

enum MSG_TYPE : uint8_t {
    MODE_OF_OPERATION = 0x00,
    COORDINATES       = 0x10,
    ALL_POSITIONS     = 0xFF
};

struct MSGALLPOSITIONS {
    uint8_t  msgTyp;
    uint16_t positions[6];
};

struct MSGOPERATIONMODE {
    uint8_t msgTyp;
    uint8_t mode;
};

struct MSGSINGLESERVO {
    uint8_t  msgTyp;
    uint8_t  servoID;
    uint16_t position;
};

struct MSGCOORDINATES {
    uint8_t msgTyp;
    float x, y, z;
};

#pragma pack(pop)

extern uint8_t ownMacAddress[6];

class ESPNow {
public:
    ESPNow() = default;

    void init();
    void sendMessage(ROLE role, const uint8_t* data, size_t len);
    void setPositionsCallback(void (*cb)(ROLE, const MSGALLPOSITIONS*));

private:
    void addPeers();
    static void readMessage(const esp_now_recv_info_t* info, const uint8_t* data, int data_len);

    static constexpr uint8_t PEER_COUNT = 3;
};
