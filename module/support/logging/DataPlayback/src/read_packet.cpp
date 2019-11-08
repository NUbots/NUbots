#include "read_packet.h"

#include <iostream>
namespace module {
namespace support {
    namespace logging {

        Packet read_packet(std::ifstream& in) {

            // Try to lock onto the next header in the stream
            int header_lock = 0;
            while (header_lock != 3) {
                int r = in.get();
                switch (r) {
                    case 0xE2: header_lock = 1; break;
                    case 0x98: header_lock = header_lock == 1 ? 2 : 0; break;
                    case 0xA2: header_lock = header_lock == 2 ? 3 : 0; break;
                    default: header_lock = 0; break;
                    case -1: throw std::runtime_error("End of file");
                }
            }

            // We are header locked! read our length
            uint32_t length = 0;
            in.read(reinterpret_cast<char*>(&length), sizeof(length));

            uint64_t timecode = 0;
            in.read(reinterpret_cast<char*>(&timecode), sizeof(timecode));

            // Build our packet!
            Packet p;
            p.timecode = std::chrono::microseconds(timecode);
            in.read(reinterpret_cast<char*>(&p.hash), sizeof(p.hash));
            p.payload.resize(length - sizeof(p.hash) - sizeof(p.timecode));
            in.read(p.payload.data(), p.payload.size());

            return p;
        }
    }  // namespace logging
}  // namespace support
}  // namespace module
