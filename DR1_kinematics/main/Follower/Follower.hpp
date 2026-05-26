#pragma once
#include <cstdint>
#include "Common/ESPNow.hpp"

class Follower {
    public:
        void init();
        void setOperationMode(OperationMode operation);
        void applyPositions(const MSGALLPOSITIONS* msg);
        void waypoint();
};
