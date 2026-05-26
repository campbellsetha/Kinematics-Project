#pragma once
#include <cstdint>
#include "Common/ESPNow.hpp"

class Primary {
    public:
        void init();
        void setOperationMode(OperationMode operation);
        void getAllPositions();
        void waypoint();
};
