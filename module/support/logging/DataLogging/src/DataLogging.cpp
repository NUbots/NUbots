#include "DataLogging.h"

#include <fmt/format.h>

#include <iomanip>
#include <sstream>

#include "extension/Configuration.h"
#include "utility/file/fileutil.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace support {
    namespace logging {

        using extension::Configuration;
        using NUClear::message::CommandLineArguments;
        using utility::support::Expression;

        std::ofstream new_output_file(const std::string& directory, const std::string& binary) {
            // Make the time into a folder pattern
            std::time_t now    = time(0);
            std::tm systemTime = *localtime(&now);

            // Create the directory for the file
            utility::file::makeDirectory(fmt::format("{}/{}", directory, binary), true);

            std::stringstream time;
            time << std::put_time(&systemTime, "%Y%m%dT%H_%M_%S");

            // Create our log file full output path
            return std::ofstream(fmt::format("{}/{}/{}.nbs", directory, binary, time.str()));
        }

        DataLogging::DataLogging(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), bytes_written(0), output_file(), handles() {

            /// This receives every DataLog message as a Sync operation (one at a time) and writes it to the file
            on<Trigger<DataLog>, Sync<DataLog>>().then([this](const DataLog& data) {
                // FILE FORMAT
                // TYPE       DATA
                // char[3]    RADIATION SYMBOL { 0xE2, 0x98, 0xA2 }
                // uint32_t   SIZE OF NEXT PACKET
                // uint64_t   TIMESTAMP DATA WAS EMITTED IN MICROSECONDS
                // uint64_t   DATA TYPE HASH

                // Convert the timestamp to a 64bit microsecond timestamp
                uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(
                                            data.timestamp.time_since_epoch())
                                            .count();

                // The size of our output timestamp hash and data
                uint32_t size = data.data.size() + sizeof(data.hash) + sizeof(timestamp_us);

                // If the file isn't open, or writing this message will exceed our max size, make a new file
                if (!output_file.is_open()
                    || (bytes_written + size + 3 + sizeof(timestamp_us) + sizeof(data.hash))
                           >= config.output.split_size) {
                    output_file.close();
                    output_file   = new_output_file(config.output.directory, config.output.binary);
                    bytes_written = 0;
                }

                // Write radiation symbol
                output_file.put(char(0xE2));
                output_file.put(char(0x98));
                output_file.put(char(0xA2));

                // Write the size of the packet
                output_file.write(reinterpret_cast<const char*>(&size), sizeof(size));

                // Write the timestamp
                output_file.write(reinterpret_cast<const char*>(&timestamp_us), sizeof(timestamp_us));

                // Write the hash
                output_file.write(reinterpret_cast<const char*>(&data.hash), sizeof(data.hash));

                // Write the actual packet data
                output_file.write(data.data.data(), data.data.size());
                output_file.flush();

                // Update the number of bytes we have written
                bytes_written += size + 3 + sizeof(timestamp_us) + sizeof(data.hash);
            });

            on<Configuration, Trigger<CommandLineArguments>, Sync<DataLog>>("DataLogging.yaml")
                .then([this](const Configuration& cfg, const CommandLineArguments& argv) {
                    {
                        // Get the details we need to generate a log file name
                        config.output.directory  = cfg["output"]["directory"].as<std::string>();
                        config.output.split_size = cfg["output"]["split_size"].as<Expression>();

                        // Get the name of the currently running binary
                        std::vector<char> data(argv[0].cbegin(), argv[0].cend());
                        data.push_back('\0');
                        const auto* base     = basename(data.data());
                        config.output.binary = std::string(base);

                        // Rescue any existing recorders that we want to keep
                        std::map<std::string, ReactionHandle> new_handles;
                        for (auto& setting : cfg["messages"].config) {
                            std::string name = setting.first.as<std::string>();
                            bool enabled     = setting.second.as<bool>();

                            // If it was enabled and we are keeping it enabled, keep it and remove it from the old list
                            if (handles.count(name) > 0 && enabled) {
                                new_handles.insert(std::make_pair(name, handles[name]));
                                handles.erase(handles.find(name));
                            }
                        }

                        // Unbind any recorders we didn't save
                        for (auto& handle : handles) {
                            log<NUClear::INFO>("Data logging for type", handle.first, "disabled");
                            handle.second.unbind();
                        }

                        // Add any new recorders that we don't have yet
                        for (auto& setting : cfg["messages"].config) {
                            std::string name = setting.first.as<std::string>();
                            bool enabled     = setting.second.as<bool>();

                            // If we are enabling this, and it wasn't already enabled, enable it
                            if (new_handles.count(name) == 0 && enabled) {
                                log<NUClear::INFO>("Data logging for type", name, "enabled");
                                new_handles.insert(std::make_pair(name, activate_recorder(name)));
                            }
                        }

                        // New handles become the handles
                        handles = std::move(new_handles);
                    }
                });
        }

    }  // namespace logging
}  // namespace support
}  // namespace module
