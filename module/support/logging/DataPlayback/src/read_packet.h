
namespace module {
namespace support {
    namespace logging {

        struct Packet {

            uint64_t hash;
            std::chrono::microseconds timecode;
            std::vector<char> payload;
        };

        inline Packet read_packet(std::ifstream& in) {

            // If something bad happens throw an exception!
            in.exceptions(std::ifstream::failbit | std::ifstream::eofbit | std::ifstream::badbit);

            // Try to lock onto the next header in the stream
            int header_lock = 0;
            for (int r = in.get(); header_lock != 3; r = in.get()) {
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


            // Build our packet!
            Packet p;
            in.read(reinterpret_cast<char*>(&p.timecode), sizeof(p.timecode));
            in.read(reinterpret_cast<char*>(&p.hash), sizeof(p.hash));
            p.payload.resize(length - sizeof(p.hash) - sizeof(p.timecode));
            in.read(p.payload.data(), p.payload.size());

            return p;
        }
    }  // namespace logging
}  // namespace support
}  // namespace module
