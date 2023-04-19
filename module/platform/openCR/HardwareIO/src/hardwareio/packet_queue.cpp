#include "HardwareIO.hpp"

namespace module::platform::openCR {

    uint8_t HardwareIO::servo_waiting() {
        // Loop through all servos in order
        for (auto& id : nugus.servo_ids()) {
            // If the length is non-zero then return
            if (packet_queue[id].size()) {
                return id;
            }
        }
        // no servos waiting
        return uint8_t(NUgus::ID::NO_ID);
    }

    int HardwareIO::opencr_waiting() {
        // just a basic wrapper for consistency
        return packet_queue[uint8_t(NUgus::ID::OPENCR)].size();
    }
}  // namespace module::platform::openCR
