#ifndef READ_PACKET_HPP
#define READ_PACKET_HPP

#include <chrono>
#include <cstdint>
#include <fstream>
#include <vector>

namespace module::support::logging {

    struct Packet {

        uint64_t hash{};
        std::chrono::microseconds timecode{};
        std::vector<uint8_t> payload;
    };

    Packet read_packet(std::ifstream& in);

}  // namespace module::support::logging

#endif  // READ_PACKET_HPP
