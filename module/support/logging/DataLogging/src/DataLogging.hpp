#ifndef MODULE_SUPPORT_LOGGING_DATALOGGING_HPP
#define MODULE_SUPPORT_LOGGING_DATALOGGING_HPP

#include <filesystem>
#include <fstream>
#include <limits>
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
            uint32_t id = 0;
            /// The raw data we will be writing
            std::vector<char> data;

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

                uint64_t max_size = std::numeric_limits<uint64_t>::max();
            } output;
        } config;

        /// Holds the reaction so we can disable it when we fill the log folder
        ReactionHandle logging_reaction{};
        /// Holds the reaction so we can disable it so we don't recheck the file size
        ReactionHandle log_check_handler{};

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
