#include <cstdio>
#include <string>
#include <iostream>
#include "ESPNow.hpp"
#include "ServoCommunication/ServoCommunication.hpp"

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

ESPNow ESP_NOW_NODE;
ServoCommunication SERVO_BUS_NODE;

int operational_mode = 1;

void set_mode(int mode){
    operational_mode = mode;
}

std::array<uint8_t, 6> get_positions (){
    double angles[6]={
        ServoCommunication.
    }
return servo_read.read_servo_positions;
}

void Teleoperation(){
    do {
        esp_now_node::send_msg(Role FLOW, getServoPositions(), len);
    } while (operational_mode == 1)
}

int getServoPositions(){

    for(int i = 0; i < 6; i++){
        SERVO_BUS_NODE.getServoPosition(i);
    }

}


