#ifndef MODULE_SUPPORT_LOGGING_DATALOGGING_H
#define MODULE_SUPPORT_LOGGING_DATALOGGING_H

#include <fstream>
#include <nuclear>

namespace module {
namespace support {
    namespace logging {

        class DataLogging : public NUClear::Reactor {
        private:
            // FILE FORMAT
            // TYPE       DATA
            // char[3]    RADIATION SYMBOL {{ 0xE2, 0x98, 0xA2 }}
            // uint32_t   SIZE OF NEXT PACKET
            // uint64_t   TIMESTAMP DATA WAS EMITTED IN MICROSECONDS
            // uint64_t   DATA TYPE HASH

            int fd              = -1;
            std::string log_dir = "";
            std::ofstream output_file;
            std::map<std::string, ReactionHandle> handles;

            struct DataLog {
                NUClear::clock::time_point timestamp;
                uint64_t hash;
                std::vector<char> data;

                DataLog() : timestamp(NUClear::clock::now()), hash(), data() {}
            };

            template <typename T>
            std::unique_ptr<DataLog> log_encode(const T& data) {

                auto log = std::make_unique<DataLog>();

                // We get the timestamp here from the time the message was emitted
                // (or now if we can't access the current reaction)
                auto task = NUClear::threading::ReactionTask::get_current_task();
                log->timestamp =
                    task ? task->stats ? task->stats->emitted : NUClear::clock::now() : NUClear::clock::now();

                // Serialise the data and get the hash for it
                log->data = NUClear::util::serialise::Serialise<T>::serialise(data);
                log->hash = NUClear::util::serialise::Serialise<T>::hash();

                return std::move(log);
            }

        public:
            /// @brief Called by the powerplant to build and setup the DataLogging reactor.
            explicit DataLogging(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace logging
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_LOGGING_DATALOGGING_H
