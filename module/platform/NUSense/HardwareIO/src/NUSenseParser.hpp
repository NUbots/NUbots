#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_NUSENSE_PARSER_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_NUSENSE_PARSER_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace module::platform::NUSense {

    struct NUSenseFrame {
        std::array<uint8_t, 3> header;  //< NBS Header, should be 0xE2, 0x98, 0xA2
        uint32_t size;                  //< Size of the payload after this field
        uint64_t timestamp;             //< Timestamp of the message
        uint64_t hash;                  //< Hash of the message type
        std::vector<uint8_t> payload;   //< The payload of the message
    };

    struct NUSenseParser {
        /**
         * Parses bytes from NUSense one byte at a time and returns a NUSenseFrame when a complete frame is read
         *
         * @param byte The next byte to parse
         *
         * @return A NUSenseFrame if a complete frame has been read, otherwise nullptr
         */
        std::unique_ptr<NUSenseFrame> operator()(const uint8_t& byte);

    private:
        /// States for the parsers state machine
        enum State { INITIAL, HEADER_1, HEADER_2, PAYLOAD, SIZE } state = INITIAL;
        /// Number of bytes to read after the size field
        uint32_t size;
        /// Buffer to store bytes read so far
        std::vector<uint8_t> buffer;
    };

}  // namespace module::platform::NUSense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_NUSENSE_PARSER_HPP
