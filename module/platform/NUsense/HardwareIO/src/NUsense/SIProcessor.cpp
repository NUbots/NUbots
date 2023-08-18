#include "SIProcessor.hpp"

#include <vector>

namespace module::platform::NUsense {
    std::vector<char> msg_to_nbs(const RawSensors& msg) {

        // Use a vector to preprocess the packet then once it is filled, we insert everything to a vector
        std::vector<char> packet_data;

        // Add header - interpret the hex values below as type char
        packet_data.emplace_back(0xE2, 0x98, 0xA2);

        // Serialise protobuf message to string then add everything to our vector
        std::vector<char> protobuf_bytes = NUClear::util::serialse(msg);
        packet_data.insert(packet_data.begin() + 3, protobuf_bytes.begin(), protobuf_bytes.end());

        return packet_data;
    }

    // TODO once NUC -> NUsense is finished do it properly
    const RawSensors& nbs_to_msg(std::vector<char> nbs_packet) {

        // Create a stream from the packet
        std::stringstream packet_stream(packet_data_str);

        // Not sure if we have to handle wrong packet headers here, but just in case
        std::vector<uint8_t> packet_data;

        uint8_t byte;
        while (packet_stream >> std::hex >> byte) {
            packet_data.push_back(byte);
        }

        // Check that our header is correct
        if (p[0] != 0xE2 || p[1] != 0x98 || p[2] != 0xA2) {
            std::cerr << "Error: invalid packet header" << std::endl;
            return 1;
        }

        std::string protobuf_packet(packet_data.begin() + 3, packet_data.end());

        RawSensors protobuf_msg;
        protobuf_msg.ParseFromString(&protobuf_packet);

        return protobuf_msg;
    }
}  // namespace module::platform::NUsense
