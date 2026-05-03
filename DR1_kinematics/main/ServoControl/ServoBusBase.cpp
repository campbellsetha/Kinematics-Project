#include "ServoBusBase.hpp"
#include <cstring>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

ServoBusBase(uart_port_t port, int tx_pin, int rx_pin)