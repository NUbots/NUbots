#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP

#include <nuclear>

#include "NUSenseParser.hpp"

#include "utility/reactor/StreamReactor.hpp"

namespace module::platform::NUsense {

    class HardwareIO : public utility::reactor::StreamReactor<HardwareIO, NUSenseParser, 5> {
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
            uint32_t size = uint32_t(payload.size() + sizeof(hash) + sizeof(timestamp_us));

            // Create the nbs packet
            std::vector<uint8_t> nbs;
            nbs.push_back(0xE2);
            nbs.push_back(0x98);
            nbs.push_back(0xA2);
            // Size
            for (size_t i = 0; i < sizeof(size); ++i) {
                nbs.push_back(uint8_t((size >> (i * 8)) & 0xFF));
            }
            // Timestamp
            for (size_t i = 0; i < sizeof(timestamp_us); ++i) {
                nbs.push_back(uint8_t((timestamp_us >> (i * 8)) & 0xFF));
            }
            // Hash
            for (size_t i = 0; i < sizeof(hash); ++i) {
                nbs.push_back(uint8_t((hash >> (i * 8)) & 0xFF));
            }
            // Payload
            nbs.insert(nbs.end(), payload.begin(), payload.end());

            // Send the packet to the device
            emit(std::make_unique<TransmitData>(nbs));
        }
    };

}  // namespace module::platform::NUsense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
