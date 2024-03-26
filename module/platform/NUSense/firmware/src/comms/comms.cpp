#include "comms.hpp"

#include <Arduino.h>

namespace comms {

    void setup() {
        /// Baud rate is ignored here as it will just run at USB speeds
        SerialUSB.begin(115200);
        while (!SerialUSB) {
            // wait for serial port to connect. Needed for native USB port only
        }
    }

    void loop() {
        for (int byte = SerialUSB.read(); byte >= 0; byte = SerialUSB.read()) {
            // Process the byte from the usb port
        }
    }

}  // namespace comms
