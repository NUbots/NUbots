#ifndef READ_PACKET_HPP
#define READ_PACKET_HPP

#include <chrono>
#include <cstdint>
#include <fstream>
#include <vector>

namespace module {
namespace support {
    namespace logging {

        struct Packet {

            uint64_t hash;
            std::chrono::microseconds timecode;
            std::vector<char> payload;
        };

        Packet read_packet(std::ifstream& in);

    }  // namespace logging
}  // namespace support
}  // namespace module

#endif  // READ_PACKET_HPP
