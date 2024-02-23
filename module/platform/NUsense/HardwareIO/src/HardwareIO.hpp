#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP

#include <nuclear>

#include "utility/reactor/StreamReactor.hpp"

namespace module::platform::NUsense {

    class HardwareIO : public utility::reactor::StreamReactor<HardwareIO, NBSParser, 5> {
    public:
        /// @brief Called by the powerplant to build and setup the NUSense reactor.
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

    private:
        template <typename T>
        void send_packet(const T& packet) {
            // Serialize the packet
            auto payload = NUClear::util::serialise::Serialise<T>::serialise(packet);
            // Get the hash of the packet
            uint64_t hash = NUClear::util::serialise::Serialise<T>::hash();

            // Get the timestamp of the emit if we can, otherwise use now
            const auto* task = NUClear::threading::ReactionTask::get_current_task();
            auto timestamp = task ? task->stats ? task->stats->emitted : NUClear::clock::now() : NUClear::clock::now();
            auto timestamp_us =
                std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();

            // Create the nbs packet
            std::vector<uint8_t> nbs;
            nbs.push_back(0xE2);
            nbs.push_back(0x98);
            nbs.push_back(0xA2);
            // Size
            for (int i = 0; i < 4; ++i) {
                nbs.push_back((payload.size() >> (i * 8)) & 0xFF);
            }
            // Timestamp
            for (int i = 0; i < 8; ++i) {
                nbs.push_back((timestamp >> (i * 8)) & 0xFF);
            }
            // Hash
            for (int i = 0; i < 8; ++i) {
                nbs.push_back((hash >> (i * 8)) & 0xFF);
            }
            // Payload
            nbs.insert(nbs.end(), payload.begin(), payload.end());

            // Send the packet to the device
            emit(std::make_unique<TransmitData>(nbs));
        }
    };

}  // namespace module::platform::NUsense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
