/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "Index.hpp"

#include <algorithm>
#include <fmt/format.h>
#include <fstream>
#include <mio/mmap.hpp>

#include "message/reflection.hpp"

namespace utility::nbs {

    namespace {
        template <typename T>
        struct SubtypeReflector;

        template <>
        struct SubtypeReflector<void> {  // NOLINT(cppcoreguidelines-special-member-functions)
            virtual uint32_t subtype(const uint8_t* payload, uint32_t length) = 0;
            virtual ~SubtypeReflector()                                       = default;
        };

        template <typename T>
        struct SubtypeReflector : public SubtypeReflector<void> {
            template <typename U = T>
            uint32_t get_subtype(...) {  // NOLINT(cert-dcl50-cpp) gimme my SFINAE!
                return 0;
            }

            template <typename U = T>
            auto get_subtype(const uint8_t* payload, uint32_t length) -> decltype(std::declval<U>().id) {
                typename U::protobuf_type pb;
                pb.ParseFromArray(payload, length);
                U msg = pb;
                return msg.id;
            }

            uint32_t subtype(const uint8_t* payload, uint32_t length) override {
                return get_subtype(payload, length);
            }
        };

        template <typename T>
        struct TimestampReflector;

        template <>
        struct TimestampReflector<void> {  // NOLINT(cppcoreguidelines-special-member-functions)
            virtual uint64_t timestamp(uint64_t original, const uint8_t* payload, uint32_t length) = 0;
            virtual ~TimestampReflector()                                                          = default;
        };

        template <typename T>
        struct TimestampReflector : public TimestampReflector<void> {

            template <typename U = T>
            auto get_timestamp(uint64_t original, ...) {  // NOLINT(cert-dcl50-cpp) gimme my SFINAE!
                // Convert the original into nanoseconds
                return original * 1000;
            }

            template <typename U = T>
            auto get_timestamp(uint64_t /*original*/, const uint8_t* payload, uint32_t length)
                -> decltype(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::declval<U>().timestamp.time_since_epoch())
                                .count()) {
                typename U::protobuf_type pb;
                pb.ParseFromArray(payload, length);
                U msg = pb;
                return std::chrono::duration_cast<std::chrono::nanoseconds>(msg.timestamp.time_since_epoch()).count();
            }

            uint64_t timestamp(uint64_t original, const uint8_t* payload, uint32_t length) override {
                return get_timestamp(original, payload, length);
            }
        };

    }  // namespace

    void build_index(const std::filesystem::path& nbs_path,
                     const std::filesystem::path& idx_path,
                     const bool& show_progress) {

        // NBS File Format
        // Name      | Type               |  Description
        // ------------------------------------------------------------
        // header    | char[3]            | NBS packet header ☢ { 0xE2, 0x98, 0xA2 }
        // length    | uint32_t           | Length of this packet after this value
        // timestamp | uint64_t           | Timestamp the data was emitted in microseconds
        // hash      | uint64_t           | the 64bit hash for the payload type
        // payload   | char[length - 16]  | the data payload
        mio::ummap_source nbs(nbs_path.string());
        zstr::ofstream idx(idx_path);

        enum State { INITIAL, HEADER_1, HEADER_2, PAYLOAD } state = INITIAL;

        std::optional<utility::support::ProgressBar> progress;
        if (show_progress) {
            progress = utility::support::ProgressBar();
        }

        // When looping, we need to at least have enough future data to have a packet and a length
        for (uint64_t p = 0; (p + 3 + sizeof(uint32_t)) < nbs.size();) {

            switch (state) {
                case INITIAL: state = nbs[p++] == 0xE2 ? HEADER_1 : INITIAL; break;
                case HEADER_1: state = nbs[p++] == 0x98 ? HEADER_2 : INITIAL; break;
                case HEADER_2: state = nbs[p++] == 0xA2 ? PAYLOAD : INITIAL; break;
                case PAYLOAD: {
                    // We always go back to initial after a payload
                    state = INITIAL;

                    // Where this packet starts
                    uint64_t offset = p - 3;

                    // Read the header of the packet
                    uint32_t size = *reinterpret_cast<uint32_t*>(&nbs[p]);

                    // We need to make sure the rest of the message is even in this file
                    // If not we need to finish here and ignore the rest of the file
                    // (leaving the half written corrupt message ignored)
                    if (p + size >= nbs.size()) {
                        p = nbs.size();
                        if (show_progress) {
                            progress.value().update(
                                p,
                                nbs.size(),
                                "B",
                                fmt::format("Reconstructing Index for {}", nbs_path.filename().string()));
                        }
                    }
                    else {
                        p += sizeof(size);
                        uint64_t timestamp = *reinterpret_cast<uint64_t*>(&nbs[p]);
                        p += sizeof(timestamp);
                        uint64_t hash = *reinterpret_cast<uint64_t*>(&nbs[p]);
                        p += sizeof(hash);

                        // Payload data
                        const uint8_t* payload  = &nbs[p];
                        uint32_t payload_length = size - sizeof(timestamp) - sizeof(hash);
                        p += payload_length;

                        // Use reflection to extract the subtype from messages that have them
                        auto sr          = message::reflection::from_hash<SubtypeReflector>(hash);
                        uint32_t subtype = sr->subtype(payload, payload_length);

                        // Use reflection to extract the timestamp from messages that have them or just return the
                        // timestamp from the nbs file if the message type doesn't have one
                        auto tr   = message::reflection::from_hash<TimestampReflector>(hash);
                        timestamp = tr->timestamp(timestamp, payload, payload_length);

                        // Write the data to the index file
                        idx.write(reinterpret_cast<char*>(&hash), sizeof(hash));
                        idx.write(reinterpret_cast<char*>(&subtype), sizeof(subtype));
                        idx.write(reinterpret_cast<char*>(&timestamp), sizeof(timestamp));
                        idx.write(reinterpret_cast<char*>(&offset), sizeof(offset));

                        // We add 3 onto the size field for the length to include the header
                        uint32_t length = size + 3 + sizeof(uint32_t);
                        idx.write(reinterpret_cast<char*>(&length), sizeof(length));

                        if (show_progress) {
                            progress.value().update(
                                p,
                                nbs.size(),
                                "B",
                                fmt::format("Reconstructing Index for {}", nbs_path.filename().string()));
                        }
                    }
                }
            }
        }
        if (show_progress) {
            progress.value().update(nbs.size(),
                                    nbs.size(),
                                    "B",
                                    fmt::format("Reconstructing Index for {}", nbs_path.filename().string()));
            progress.value().close();
        }
    }

    IndexItemFile* Index::begin() {
        return idx.begin();
    }

    IndexItemFile* Index::end() {
        return idx.end();
    }

}  // namespace utility::nbs
