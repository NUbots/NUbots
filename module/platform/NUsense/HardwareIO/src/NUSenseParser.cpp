#include "NUSenseParser.hpp"

namespace module::platform::NUsense {

    /**
     * Read a little endian value from a byte array
     *
     * @tparam T The type of the value to read
     *
     * @param ptr The pointer to the first byte of the value
     *
     * @return The value read from the byte array
     */
    template <typename T>
    T read_le(const uint8_t* ptr) {
        T value = 0;
        for (size_t i = 0; i < sizeof(T); i++) {
            value |= static_cast<T>(ptr[i]) << (8 * i);
        }
        return value;
    }

    std::unique_ptr<NUSenseFrame> NUSenseParser::operator()(const uint8_t& byte) {
        if (state == INITIAL) {
            buffer.clear();
        }

        buffer.push_back(byte);
        switch (state) {
            case INITIAL: state = byte == 0xE2 ? HEADER_1 : INITIAL; break;
            case HEADER_1: state = byte == 0x98 ? HEADER_2 : INITIAL; break;
            case HEADER_2: state = byte == 0xA2 ? SIZE : INITIAL; break;
            case SIZE: {
                // We have read the size of the payload
                if (buffer.size() == 7) {
                    size  = read_le<uint32_t>(&buffer[3]);
                    state = PAYLOAD;
                }
            } break;
            case PAYLOAD: {
                // Read all the bytes of this packet
                if (buffer.size() == size + 3 + sizeof(uint32_t)) {
                    // We always go back to initial after a payload
                    state = INITIAL;

                    auto msg       = std::make_unique<NUSenseFrame>();
                    msg->header    = {buffer[0], buffer[1], buffer[2]};
                    msg->size      = size;
                    msg->timestamp = read_le<uint64_t>(&buffer[3]);
                    msg->hash      = read_le<uint64_t>(&buffer[11]);
                    msg->payload   = std::vector<uint8_t>(std::next(buffer.begin(), 19), buffer.end());

                    return msg;
                }
            }
        }

        return nullptr;
    }

}  // namespace module::platform::NUsense
