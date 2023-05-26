#include "HardwareIO.hpp"

namespace module::platform::OpenCR {

    /**
     * @todo make a PacketQueue class with these as memeber functions and add wrappers
     * for queue.push() and queue.pop() to clean everything up.
     */

    NUgus::ID HardwareIO::servo_waiting() {
        // Loop through all servos in order
        for (const auto& id : nugus.servo_ids()) {
            // If the length is non-zero then return
            if (!packet_queue[NUgus::ID(id)].empty()) {
                return NUgus::ID(id);
            }
        }
        // no servos waiting
        return NUgus::ID::NO_ID;
    }

    bool HardwareIO::opencr_waiting() {
        // just a basic wrapper for consistency
        return !packet_queue[NUgus::ID::OPENCR].empty();
    }

    NUgus::ID HardwareIO::queue_item_waiting() {
        // Loop through all devices in the queue
        for (const auto& keypair : packet_queue) {
            // If any are waiting, then return their ID
            if (keypair.second.size()) {
                return keypair.first;
            }
        }
        // no devices waiting
        return NUgus::ID::NO_ID;
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

}  // namespace module::platform::OpenCR
