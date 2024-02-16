#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP

#include <nuclear>

#include "utility/reactor/StreamReactor.hpp"

namespace module::platform::NUsense {

    class HardwareIO : public utility::reactor::StreamReactor<HardwareIO, NBSParser, 5> {
    public:
        /// @brief Called by the powerplant to build and setup the GPS reactor.
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

    private:
        template <typename T>
        void send_packet(const T& packet) {
            // Serialize the packet
            auto payload = NUClear::util::serialise::Serialise<T>::serialise(packet);

            // Create the nbs packet
            std::vector<uint8_t> nbs;
            bytes.push_back(0xE2);
            bytes.push_back(0x98);
            bytes.push_back(0xA2);
            // Size
            for (int i = 0; i < 4; ++i) {
                bytes.push_back((payload.size() >> (i * 8)) & 0xFF);
            }
            // Timestamp
            for (int i = 0; i < 8; ++i) {
                bytes.push_back((timestamp >> (i * 8)) & 0xFF);
            }
            // Hash
            for (int i = 0; i < 8; ++i) {
                bytes.push_back((hash >> (i * 8)) & 0xFF);
            }
            // Payload
            bytes.insert(bytes.end(), payload.begin(), payload.end());

            // Send the packet to the device
            emit(std::make_unique<TransmitData>(packet.bytes));
        }
    };

}  // namespace module::platform::NUsense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
