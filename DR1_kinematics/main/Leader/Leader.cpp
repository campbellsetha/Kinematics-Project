#include <cstdio>
#include <string>
#include <iostream>
#include "common/esp_now_node.hpp"

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_log.h"
    #include "driver/usb_serial_jtag.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#include <string>

}

esp_now_node ESP_NOW_NODE;

int operational_mode = 1;

void set_mode(int mode){
    operational_mode = mode;
}

std::array<uint8_t, 6> get_positions (){
    double angles[6]={
        Servo
    }
return servo_read.read_servo_positions;
}

void Teleoperation(){
    do {
        esp_now_node::send_msg(Role FLOW, get_positions(), len);
    } while (operational_mode == 1)
}


