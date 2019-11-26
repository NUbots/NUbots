#ifndef MODULE_SUPPORT_LOGGING_DATALOGGING_H
#define MODULE_SUPPORT_LOGGING_DATALOGGING_H

#include <fstream>
#include <nuclear>

namespace module {
namespace support {
    namespace logging {

        class DataLogging : public NUClear::Reactor {
        private:
            struct DataLog {
                /// The timestamp this message was emitted at
                NUClear::clock::time_point timestamp;
                /// The type hash generated via NUClear serialise
                uint64_t hash;
                /// The raw data we will be writing
                std::vector<char> data;

                DataLog() : timestamp(NUClear::clock::now()), hash(), data() {}
            };

            /// Activate a recorder for a specific type and return the ReactionHandle associated with it
            ReactionHandle activate_recorder(const std::string& msg);

        public:
            /// @brief Called by the powerplant to build and setup the DataLogging reactor.
            explicit DataLogging(std::unique_ptr<NUClear::Environment> environment);

        private:
            struct {
                struct {
                    /// The directory that we are saving the file to
                    std::string directory;
                    /// The name of the binary (role) that we are running
                    std::string binary;
                    /// The threshold of bytes where after this we split the file
                    uint64_t split_size;
                } output;
            } config;

            /// The number of bytes written to the file so far
            uint64_t bytes_written;
            /// The file we are outputting to currently
            std::ofstream output_file;
            /// The reaction handles for types we have currently bound that we are recording
            std::map<std::string, ReactionHandle> handles;
        };
    }  // namespace logging
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_LOGGING_DATALOGGING_H
