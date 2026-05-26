#pragma once
#include <cstdint>
#include "Common/ESPNow.hpp"

class Leader {
    public:
        void init();
        void setOperationMode(OperationMode operation);
        void getPositions();
        void waypoint();
};
