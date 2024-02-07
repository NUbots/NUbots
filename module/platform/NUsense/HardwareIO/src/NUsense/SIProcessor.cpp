#include "SIProcessor.hpp"

#include <vector>

namespace module::platform::NUsense {
    std::vector<uint8_t> msg_to_nbs(const ServoTargets& msg) {

        // Use a vector to preprocess the packet then once it is filled, we insert everything to a vector
        std::vector<uint8_t> packet_data;

        // Add header - interpret the hex values below as type char
        packet_data.emplace_back(0xE2);
        packet_data.emplace_back(0x98);
        packet_data.emplace_back(0xA2);

        // Serialise protobuf message to string, add its length as 4 uint8_t's (1024 bits) then add everything else
        std::vector<uint8_t> protobuf_bytes = NUClear::util::serialise::Serialise<ServoTargets>::serialise(msg);
        uint32_t msg_length = static_cast<std::uint32_t>(protobuf_bytes.size());

        std::vector<uint8_t> high_byte_low_byte = {static_cast<uint8_t>((msg_length >> 8) & 0x00FF), static_cast<uint8_t>(msg_length & 0x00FF)};
        packet_data.insert(packet_data.begin() + 3, high_byte_low_byte.begin(), high_byte_low_byte.end());
        packet_data.insert(packet_data.begin() + 3 + high_byte_low_byte.size(), protobuf_bytes.begin(), protobuf_bytes.end());

        return packet_data;
    }

    // TODO once NUC -> NUsense is finished do it properly
    // const RawSensors& nbs_to_msg(std::vector<char> nbs_packet) {

        // // Create a stream from the packet
        // std::stringstream packet_stream(packet_data_str);

        // // Not sure if we have to handle wrong packet headers here, but just in case
        // std::vector<uint8_t> packet_data;

        // uint8_t byte;
        // while (packet_stream >> std::hex >> byte) {
        //     packet_data.push_back(byte);
        // }

        // // Check that our header is correct
        // if (p[0] != 0xE2 || p[1] != 0x98 || p[2] != 0xA2) {
        //     std::cerr << "Error: invalid packet header" << std::endl;
        //     return 1;
        // }

        // std::string protobuf_packet(packet_data.begin() + 3, packet_data.end());

        // RawSensors protobuf_msg;
        // protobuf_msg.ParseFromString(&protobuf_packet);
    //     RawSensors rs();
    //     return rs;
    // }
}  // namespace module::platform::NUsense
