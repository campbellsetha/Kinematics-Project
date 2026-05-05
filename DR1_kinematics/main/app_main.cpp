#include <cstdio>
#include <cstring>
#include <iostream>

extern "C" {
    #include "nvs_flash.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_log.h"
    #include "driver/usb_serial_jtag.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"

}

main(){
    int choice;
    welcome();
    
    switch(choice) {
        case 1: teleoperation();
        case 2: waypoint();
        default: std::cout << "Invalid choice. Try Again.\n";
    }


}



static void telemetry_in(const esp_now_recv_info_t info, const int data, int len) {

}

void welcome() {
    std::cout << "\n_-_-_WELCOME_-_-_\n";
    std::cout << "What would you like to try?";
    std::cout << "1] Teleoperation";
    std::cout << "2] Waypoint Operation";
    std::cin >> choice;

}

void teleoperation() {

}
