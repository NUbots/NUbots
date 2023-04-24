#include "SIProcessor.hpp"

#include <vector>

namespace NUsense {
    std::string msg_to_nbs(message::platform::RawSensorsV2& msg) {

        // Use a vector to preprocess the packet then once it is filled,
        // We insert everything to a string stream
        std::vector<uint8_t> packets;

        // Add header
        packet_data.push_back(0xE2);
        packet_data.push_back(0x98);
        packet_data.push_back(0xA2);

        // Serialise protobuf message to string then add everything to our vector
        std::string protobuf_bytes = msg.SerializeToString();

        for (const auto& byte : protobuf_bytes) {
            packet_data.push_back(static_cast<uint8_t>(byte));
        }

        // Add packets to string stream then convert to string
        std::stringstream nbs_msg;
        for (const auto& byte : packet_data) {
            nbs_msg << std::hex << static_cast<int>(byte);
        }

        return nbs_msg.str();
    }

    message::platform::RawSensorsV2 nbs_to_msg(std::string nbs_packet) {

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

        message::platform::RawSensorsV2 protobuf_msg;
        protobuf_msg.ParseFromString(&protobuf_packet);

        return protobuf_msg;
    }
}  // namespace NUsense
