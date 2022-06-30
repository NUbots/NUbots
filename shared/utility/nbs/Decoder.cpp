#include "Decoder.hpp"

#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <mio/mmap.hpp>
#include <nuclear>
#include <numeric>
#include <vector>

#include "Index.hpp"

namespace utility::nbs {

    Decoder::Decoder(
        const std::vector<std::filesystem::path>& paths,
        const std::function<void(const std::filesystem::path&, const uint64_t&, const uint64_t&)>& progress)
        : index(paths, progress) {

        // Memory map all the files so we can access the data
        for (const auto& path : paths) {
            // Memory map the file into ram
            mmaps.emplace_back(path.string());
        }
    }

    Decoder::Decoder(
        const std::filesystem::path& path,
        const std::function<void(const std::filesystem::path&, const uint64_t&, const uint64_t&)>& progress)
        : Decoder(std::vector<std::filesystem::path>({path}), progress) {}

    void Decoder::process(
        const std::function<void(const uint64_t&, const uint64_t&, const uint64_t&, const uint64_t&)>& progress) {

        // Work out the total number of bytes to be handled from all the files
        uint64_t total_bytes =
            std::accumulate(mmaps.begin(),
                            mmaps.end(),
                            uint64_t(0),
                            [&](const uint64_t& a, const mio::ummap_source& b) { return a + b.size(); });
        uint64_t total_messages = std::distance(index.begin(), index.end());

        // Loop through the index
        uint64_t current_bytes   = 0;
        uint64_t current_message = 0;
        for (const auto& i : index) {

            // Skip decoding packets that we don't have a callback for
            if (callbacks.count(i.type) != 0) {
                // Where our data is in mapped memory
                const uint8_t* data = &mmaps[i.fileno][i.offset];

                // Read out the length from the packet
                uint32_t length    = *reinterpret_cast<const uint32_t*>(data + 3);
                uint64_t timestamp = *reinterpret_cast<const uint64_t*>(data + 3 + sizeof(length));

                // Find the begining and end of the payload section
                // Offset + 3 (header) + uint32_t (length) + uint64_t (timestamp) + uint64_t (hash)
                const uint8_t* payload  = data + 3 + sizeof(length) + sizeof(timestamp) + sizeof(uint64_t);
                uint64_t payload_length = length - sizeof(timestamp) - sizeof(uint64_t);

                // Get the timestamp out from the index
                NUClear::clock::time_point index_timestamp{std::chrono::nanoseconds(i.timestamp)};
                NUClear::clock::time_point emit_timestamp{std::chrono::microseconds(timestamp)};

                callbacks[i.type](emit_timestamp, index_timestamp, payload, payload_length);
            }

            // Update our progress callback
            current_bytes += i.length;
            current_message += 1;
            if (progress) {
                progress(current_message, total_messages, current_bytes, total_bytes);
            }
        }
    }

}  // namespace utility::nbs
