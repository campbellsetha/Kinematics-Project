#pragma once

#include <cstdint>
#include <cstddef>

extern "C" {
    #include "driver/uart.h"
    #include "esp_log.h"
}

class ServoBusBase {
    public:
        explicit ServoBusBase(uart_port_t port = UART_NUM_1, int tx_pin = 21, int rx_pin = 20);
        void init();
    protected
}