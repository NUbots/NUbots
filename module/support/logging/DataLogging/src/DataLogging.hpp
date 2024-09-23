/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#ifndef MODULE_SUPPORT_LOGGING_DATALOGGING_HPP
#define MODULE_SUPPORT_LOGGING_DATALOGGING_HPP

#include <filesystem>
#include <fstream>
#include <nuclear>

#include "utility/nbs/Encoder.hpp"

namespace module::support::logging {

    class DataLogging : public NUClear::Reactor {
    public:
        struct DataLog {
            /// The timestamp this message was emitted at
            NUClear::clock::time_point timestamp;
            /// The timestamp inside the message or, if message does not exist, the emit timestamp
            uint64_t message_timestamp{};
            /// The type hash generated via NUClear serialise
            uint64_t hash{};
            /// The id field of the message or, if id does not exist, 0.
            uint32_t subtype = 0;
            /// The raw data we will be writing
            std::vector<uint8_t> data;

            DataLog() : timestamp(NUClear::clock::now()) {}
        };

        /// Activate a recorder for a specific type and return the ReactionHandle associated with it
        ReactionHandle activate_recorder(const std::string& name);

        /// @brief Called by the powerplant to build and setup the DataLogging reactor.
        explicit DataLogging(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            struct {
                /// The directory that we are saving the file to
                std::filesystem::path directory;
                /// The name of the binary (role) that we are running
                std::string binary;
                /// The threshold of bytes where after this we split the file
                uint64_t split_size{};
                /// The encryption passphrase to use or empty if not encrypting
                std::string passphrase;
            } output;
        } config;

        /// The file we are outputting to currently
        std::filesystem::path output_file_path{};
        /// The file we are outputting our index to currently
        std::filesystem::path index_file_path{};
        /// The encoder that writes the files
        /// Using a unique pointer as Encoder does not have a default constructor
        std::unique_ptr<utility::nbs::Encoder> encoder;

        /// The reaction handles for types we have currently bound that we are recording
        std::map<std::string, ReactionHandle> handles;
    };
}  // namespace module::support::logging

#endif  // MODULE_SUPPORT_LOGGING_DATALOGGING_HPP
