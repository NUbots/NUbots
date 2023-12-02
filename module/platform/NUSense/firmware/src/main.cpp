#include <Arduino.h>

#include "battery/battery.hpp"
#include "comms/comms.hpp"
#include "dynamixel/dynamixel.hpp"
#include "imu/imu.hpp"

void setup() {
    comms::setup();
    dynamixel::setup();
    imu::setup();
    battery::setup();
}

void loop() {
    comms::loop();
    dynamixel::loop();
    imu::loop();
    battery::loop();
}
