#include "HardwareIO.hpp"

namespace module::platform::openCR {

    /**
     * @todo make a PacketQueue class with these as memeber functions and add wrappers
     * for queue.push() and queue.pop() to clean everything up.
     */

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

    uint8_t HardwareIO::opencr_waiting() {
        // just a basic wrapper for consistency
        return packet_queue[uint8_t(NUgus::ID::OPENCR)].size();
    }

    uint8_t HardwareIO::queue_item_waiting() {
        // Loop through all devices in the queue
        for (const auto& keypair : packet_queue) {
            // If any are waiting, then return their ID
            if (keypair.second.size()) {
                return keypair.first;
            }
        }
        // no devices waiting
        return uint8_t(NUgus::ID::NO_ID);
    }

    int HardwareIO::queue_clear_all() {
        int packets_cleared = 0;
        // Loop through all initialised queues
        for (const auto& keypair : packet_queue) {
            // Add the packets to the running total
            packets_cleared += keypair.second.size();
            // Clear the queue
            packet_queue[keypair.first].clear();
        }
        return packets_cleared;
    }

}  // namespace module::platform::openCR
