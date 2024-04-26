#include "NUSenseParser.hpp"

namespace module::platform::NUSense {

    uint32_t read_le_32(const uint8_t* ptr) {
        return (uint32_t(ptr[0]) << 0) | (uint32_t(ptr[1]) << 8) | (uint32_t(ptr[2]) << 16) | (uint32_t(ptr[3]) << 24);
    }

    uint64_t read_le_64(const uint8_t* ptr) {
        return (uint64_t(ptr[0]) << 0) | (uint64_t(ptr[1]) << 8) | (uint64_t(ptr[2]) << 16) | (uint64_t(ptr[3]) << 24)
               | (uint64_t(ptr[4]) << 32) | (uint64_t(ptr[5]) << 40) | (uint64_t(ptr[6]) << 48)
               | (uint64_t(ptr[7]) << 56);
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
                    size  = read_le_32(&buffer[3]);
                    state = PAYLOAD;
                }
            } break;
            case PAYLOAD: {
                // Read all the bytes of this packet
                if (buffer.size() == size + 3 + sizeof(uint32_t)) {
                    // We always go back to initial after a payload
                    state = INITIAL;

                    auto msg = std::make_unique<NUSenseFrame>();

                    msg->header = {buffer[0], buffer[1], buffer[2]};
                    msg->size   = size;

                    uint32_t buf_idx = sizeof(msg->header) + sizeof(size);
                    msg->timestamp   = read_le_64(&buffer[buf_idx]);

                    buf_idx += sizeof(msg->timestamp);
                    msg->hash = read_le_64(&buffer[buf_idx]);

                    buf_idx += sizeof(msg->hash);
                    msg->payload = std::vector<uint8_t>(std::next(buffer.begin(), buf_idx), buffer.end());

                    return msg;
                }
            }
        }

        return nullptr;
    }

}  // namespace module::platform::NUsense
