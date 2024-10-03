/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#ifndef UTILITY_NBS_DECODER_HPP
#define UTILITY_NBS_DECODER_HPP

#include <filesystem>
#include <functional>
#include <map>
#include <mio/mmap.hpp>
#include <nuclear>
#include <vector>

#include "Index.hpp"

namespace utility::nbs {

    class Decoder {
    public:
        struct Iterator {

            struct Type {
                [[nodiscard]] bool has_callback() const {
                    return decoder.has_callback(item->item.type) || decoder.has_unhandled_callback()
                           || decoder.has_all_callback();
                }

                void operator()() const {
                    decoder.run_callbacks(item);
                }

                const IndexItemFile* item;

                Type(const IndexItemFile* item, Decoder& decoder) : item(item), decoder(decoder) {}

            private:
                Decoder& decoder;
            };

            using iterator_category = std::random_access_iterator_tag;
            using value_type        = Type;
            using difference_type   = std::ptrdiff_t;
            using pointer           = value_type*;
            using reference         = value_type&;

            Iterator& operator++() {
                ++item;
                return *this;
            }
            // NOLINTNEXTLINE(readability-const-return-type,cert-dcl21-cpp)
            Iterator operator++(int) {
                Iterator it = *this;
                ++(*this);
                return it;
            }
            value_type operator*() const {
                return {item, decoder};
            }
            value_type operator->() {
                return {item, decoder};
            }
            Iterator& operator--() {
                --item;
                return *this;
            }
            // NOLINTNEXTLINE(readability-const-return-type,cert-dcl21-cpp)
            Iterator operator--(int) {
                Iterator it = *this;
                --(*this);
                return it;
            }
            Iterator& operator+=(const difference_type& n) {
                item += n;
                return *this;
            }
            Iterator operator+(const difference_type& n) const {
                Iterator it = *this;
                it += n;
                return it;
            }
            Iterator& operator-=(const difference_type& n) {
                item -= n;
                return *this;
            }
            Iterator operator-(const difference_type& n) const {
                Iterator it = *this;
                it -= n;
                return it;
            }
            difference_type operator-(const Iterator& other) const {
                return item - other.item;
            }

            std::strong_ordering operator<=>(const Iterator& other) const {
                return item <=> other.item;
            }
            bool operator==(const Iterator& other) const {
                return item == other.item;
            }

            Iterator& operator=(const Iterator& other) {
                if (this != &other) {
                    item = other.item;
                }
                return *this;
            }

            Iterator(Iterator&& other) noexcept : item(other.item), decoder(other.decoder) {
                other.item = nullptr;
            }

            Iterator& operator=(Iterator&& other) noexcept {
                if (this != &other) {
                    item       = other.item;
                    other.item = nullptr;
                }
                return *this;
            }

            Iterator(Decoder& decoder, const IndexItemFile* index_it) : item(index_it), decoder(decoder) {}

            Iterator(const Iterator& other) = default;

            ~Iterator() = default;

        private:
            const IndexItemFile* item;
            Decoder& decoder;
        };

        Iterator begin() {
            return {(*this), index.begin()};
        }

        Iterator end() {
            return {(*this), index.end()};
        }


        /**
         * @brief Default constructor for the Decoder.
         */
        Decoder() = default;

        /**
         * Constructs a new Decoder for the given path
         *
         * @param path          The path to the nbs file to decode
         * @param show_progress If the progress of loading the nbs index should be shown
         */
        Decoder(const std::filesystem::path& path, const bool& show_progress = false);

        /**
         * Constructs a new Decoder for the given path
         *
         * @param path          The path to the nbs files to decode
         * @param show_progress If the progress of loading the nbs index should be shown
         */
        Decoder(const std::vector<std::filesystem::path>& paths, const bool& show_progress = false);

        /**
         * Sets the callback for a specific type for the decoder.
         *
         * If you call this function again with a new callback for the same message type it will override the old
         * callback that was set.
         *
         * The arguments that are passed to this callback are the following:
         *
         * const NUClear::clock::time_point& The timestamp that was extracted from the nbs files emit time (when the
         *                                   message was emitted)
         *
         * const NUClear::clock::time_point& The timestamp that was provided from the nbs index, this will be the
         *                                   timestamp that was stored in the message, or if unavailable the timestamp
         *                                   from the nbs file's emit time
         *
         * const uint8_t*                    The pointer to the memory mapped location where the packet begins
         *
         * const uint32_t&                   The length of the packet that is stored in that memory mapped location
         *
         * @tparam MessageType The message type to set the callback for
         *
         * @param callback The callback to set for this specific message type
         */
        template <typename MessageType>
        void on(const std::function<void(const NUClear::clock::time_point& /*emit_time*/,
                                         const NUClear::clock::time_point& /*index_time*/,
                                         const uint8_t* /*payload*/,
                                         const uint32_t& /*length*/)>& callback) {
            // Add a callback that will deserialise the message and provide it to the callback
            callbacks[NUClear::util::serialise::Serialise<MessageType>::hash()] = callback;
        }

        template <typename MessageType>
        void on(const std::function<void(const NUClear::clock::time_point& /*emit_time*/,
                                         const NUClear::clock::time_point& /*index_time*/,
                                         const MessageType& /*msg*/)>& callback) {
            // Add a callback that will deserialise the message and provide it to the callback
            on<MessageType>([callback](const NUClear::clock::time_point& emit_time,
                                       const NUClear::clock::time_point& index_time,
                                       const uint8_t* payload,
                                       const uint32_t& length) {
                typename MessageType::protobuf_type pb;
                pb.ParseFromArray(payload, length);

                // Call the callback with the deserialised message
                callback(emit_time, index_time, MessageType(pb));
            });
        }

        template <typename MessageType>
        void on(const std::function<void(const NUClear::clock::time_point& /*emit_time*/, const MessageType& /*msg*/)>&
                    callback) {
            on<MessageType>([callback](const NUClear::clock::time_point& emit_time,
                                       const NUClear::clock::time_point& /*index_time*/,
                                       const MessageType& msg) { callback(emit_time, msg); });
        }

        template <typename MessageType>
        void on(const std::function<void(const MessageType& /*msg*/)>& callback) {
            on<MessageType>([callback](const NUClear::clock::time_point& /*emit_time*/,
                                       const NUClear::clock::time_point& /*index_time*/,
                                       const MessageType& msg) { callback(msg); });
        }

        /**
         * Sets the callback for all messages for the decoder that are not specified in other callbacks
         *
         * The arguments that are passed to this callback are the following:
         *
         * const uint64_t& emit_timestamp  The timestamp the message was originally emitted at
         * const uint64_t& index_timestamp The timestamp inside the message
         * const uint64_t& hash_type       The type hash of the message
         * const uint32_t& subtype         The subtype of the message
         * const void*     payload         A pointer to the raw memory that holds the payload data of the message
         * const size_t&   payload_length  The number of bytes in the payload pointer
         *
         * @param callback A callback function to process all messages that don't have a registered callback
         */
        void unhandled(const std::function<void(const uint64_t& emit_timestamp,
                                                const uint64_t& index_timestamp,
                                                const uint64_t& hash_type,
                                                const uint32_t& subtype,
                                                const void* payload,
                                                const size_t& payload_length)>& callback);

        /**
         * Sets the callback for all messages for the decoder
         *
         * The arguments that are passed to this callback are the following:
         *
         * const uint64_t& emit_timestamp  The timestamp the message was originally emitted at
         * const uint64_t& index_timestamp The timestamp inside the message
         * const uint64_t& hash_type       The type hash of the message
         * const uint32_t& subtype         The subtype of the message
         * const void*     payload         A pointer to the raw memory that holds the payload data of the message
         * const size_t&   payload_length  The number of bytes in the payload pointer
         *
         * @param callback A callback function to process all messages
         */
        void all(const std::function<void(const uint64_t& emit_timestamp,
                                          const uint64_t& index_timestamp,
                                          const uint64_t& hash_type,
                                          const uint32_t& subtype,
                                          const void* payload,
                                          const size_t& payload_length)>& callback);

        /**
         * Process the loaded files using the current callbacks.
         *
         * This will go through the entire file in timestamp order and call the callbacks for each message.
         * This is done single threaded so if any multiprocessing is to be done it will have to happen from within the
         * callbacks.
         *
         * The callback to this function will constantly be fed the current and total number of bytes as the file is
         * being processed. This can be used to monitor the progress and update a progress bar.
         *
         * @param progress The callback that will be provided with message and byte progress updates through the file
         *                 with arguments as follows:
         *                     const uint64_t& The current amount of messages processed
         *                     const uint64_t& The total amount of messages
         *                     const uint64_t& The current amount of bytes processed
         *                     const uint64_t& The total amount of bytes
         */
        void process(const std::function<void(const uint64_t& /*current_message*/,
                                              const uint64_t& /*total_messages*/,
                                              const uint64_t& /*current_bytes*/,
                                              const uint64_t& /*total_bytes*/)>& progress = {});

        /**
         * Checks if a callback with the given hash exists.
         *
         * @param hash The hash value of the callback.
         * @return True if a callback with the given hash exists, false otherwise.
         */
        bool has_callback(const uint64_t& hash) const {
            return callbacks.contains(hash);
        }

        /**
         * Checks if the decoder has a callback function for all events.
         *
         * @return true if a callback function for all events is set, false otherwise.
         */
        bool has_all_callback() const {
            return all_callback.operator bool();
        }

        /**
         * @brief Checks if there is an unhandled callback.
         *
         * @return true if there is an unhandled callback, false otherwise.
         */
        bool has_unhandled_callback() const {
            return unhandled_callback.operator bool();
        }

        /**
         * Runs the callbacks for the given IndexItemFile.
         *
         * @param i The IndexItemFile containing the data to process.
         */
        void run_callbacks(const IndexItemFile* i) {
            // Where the data is in mapped memory
            const uint8_t* data = &mmaps[i->fileno][i->item.offset];

            // Read out the length from the packet
            uint32_t length    = *reinterpret_cast<const uint32_t*>(data + 3);
            uint64_t timestamp = *reinterpret_cast<const uint64_t*>(data + 3 + sizeof(length));

            // Find the beginning and end of the payload section
            // Offset + 3 (header) + uint32_t (length) + uint64_t (timestamp) + uint64_t (hash)
            const uint8_t* payload  = data + 3 + sizeof(length) + sizeof(timestamp) + sizeof(uint64_t);
            uint64_t payload_length = length - sizeof(timestamp) - sizeof(uint64_t);

            // Check for a message-specific callback, if one exists use that for processing this message
            if (callbacks.contains(i->item.type)) {
                // Get the timestamp out from the index
                NUClear::clock::time_point index_timestamp{std::chrono::nanoseconds(i->item.timestamp)};
                NUClear::clock::time_point emit_timestamp{std::chrono::microseconds(timestamp)};

                callbacks[i->item.type](emit_timestamp, index_timestamp, payload, payload_length);
            }
            // Use the catch-unhandled callback if a message-specific callback hasn't been registered
            else if (unhandled_callback) {
                unhandled_callback(timestamp,
                                   i->item.timestamp,
                                   i->item.type,
                                   i->item.subtype,
                                   payload,
                                   payload_length);
            }

            // Use the catch-all callback for all messages
            if (all_callback) {
                all_callback(timestamp, i->item.timestamp, i->item.type, i->item.subtype, payload, payload_length);
            }
        }

    private:
        /// The index that has been constructed from the loaded nbs files
        Index index;
        /// The memory mapped IO nbs files
        std::vector<mio::ummap_source> mmaps{};
        /// The map of callbacks that will be executed when a message of the appropriate hash type is found
        std::map<uint64_t,
                 std::function<void(const NUClear::clock::time_point& /*emit_time*/,
                                    const NUClear::clock::time_point& /*index_time*/,
                                    const uint8_t* /*payload*/,
                                    const uint32_t& /*length*/)>>
            callbacks{};
        /// A callback function that will be executed with a hash type not found by the other callbacks
        std::function<void(const uint64_t& emit_timestamp,
                           const uint64_t& index_timestamp,
                           const uint64_t& hash_type,
                           const uint32_t& subtype,
                           const void* payload,
                           const size_t& payload_length)>
            unhandled_callback{};
        /// A callback function that will be executed for all has types
        std::function<void(const uint64_t& emit_timestamp,
                           const uint64_t& index_timestamp,
                           const uint64_t& hash_type,
                           const uint32_t& subtype,
                           const void* payload,
                           const size_t& payload_length)>
            all_callback{};
    };

}  // namespace utility::nbs

#endif  // UTILITY_NBS_DECODER_HPP
