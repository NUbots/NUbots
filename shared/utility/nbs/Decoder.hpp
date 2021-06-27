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
        Decoder(
            const std::filesystem::path& path,
            const std::function<void(const std::filesystem::path&, const uint64_t&, const uint64_t&)>& progress = {});
        Decoder(
            const std::vector<std::filesystem::path>& paths,
            const std::function<void(const std::filesystem::path&, const uint64_t&, const uint64_t&)>& progress = {});

        /**
         * @brief Sets the callback for a specific type for the decoder.
         *
         * If you call this function again with a new callback for the same message type it will override the old
         * callback that was set.
         *
         * The arguments that are passed to this callback are the following:
         *
         *      NUClear::clock::time_point The timestamp that was extracted from the nbs files emit time (when the
         *      message was emitted)
         *
         *      NUClear::clock::time_point The timestamp that was provided from the nbs index, this will be the
         *      timestamp that was stored in the message, or if unavailable the timestamp from the nbs file's emit time
         *
         *      const uint8_t* the pointer to the memory mapped location where the packet begins
         *
         *      const uint32_t& the length of the packet that is stored in that memory mapped location
         *
         * @tparam MessageType the message type to set the callback for
         *
         * @param callback  the callback to set for this specific message type
         */
        template <typename MessageType>
        void on(const std::function<void(const NUClear::clock::time_point&,
                                         const NUClear::clock::time_point&,
                                         const uint8_t*,
                                         const uint32_t&)>& callback) {

            // Add a callback that will deserialise the message and provide it to the callback
            callbacks[NUClear::util::serialise::Serialise<MessageType>::hash()] = callback;
        }

        template <typename MessageType>
        void on(const std::function<void(const NUClear::clock::time_point&,
                                         const NUClear::clock::time_point&,
                                         const MessageType&)>& callback) {

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
        void on(const std::function<void(const NUClear::clock::time_point&, const MessageType&)>& callback) {
            on<MessageType>([callback](const NUClear::clock::time_point& emit_time,
                                       const NUClear::clock::time_point& /*index_time*/,
                                       const MessageType& msg) { callback(emit_time, msg); });
        }
        template <typename MessageType>
        void on(const std::function<void(const MessageType&)>& callback) {
            on<MessageType>([callback](const NUClear::clock::time_point& /*index_time*/,
                                       const NUClear::clock::time_point& /*emit_time*/,
                                       const MessageType& msg) { callback(msg); });
        }

        /**
         * @brief Process the loaded files using the current callbacks.
         *
         * This will go through the entire file in timestamp order and call the callbacks for each message.
         * This is done single threaded so if any multiprocessing is to be done it will have to happen from within the
         * callbacks.
         *
         * The callback to this function will constantly be fed the current and total number of bytes that we have moved
         * through the file. As well as the current and total number of messages we have moved through the file. This
         * can be used to monitor the progress and update a progress bar.
         *
         * @param progress  the callback that will be provided with byte progress updates through the file
         */
        void process(const std::function<void(const uint64_t&, const uint64_t&, const uint64_t&, const uint64_t&)>&
                         progress = {});


    private:
        /// The index that has been constructed from the loaded nbs files
        Index index;
        /// The memory mapped IO nbs files
        std::vector<mio::ummap_source> mmaps;
        /// The map of callbacks that will be executed when a message of the appropriate hash type is found
        std::map<uint64_t,
                 std::function<void(const NUClear::clock::time_point&,
                                    const NUClear::clock::time_point&,
                                    const uint8_t*,
                                    const uint32_t&)>>
            callbacks;
    };

}  // namespace utility::nbs

#endif  // UTILITY_NBS_DECODER_HPP
