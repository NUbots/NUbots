#include "DataLogging.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <zstr.hpp>

#include "extension/Configuration.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::support::logging {

    using extension::Configuration;
    using NUClear::message::CommandLineArguments;
    using utility::support::Expression;

    std::string formatted_time() {
        std::time_t now     = time(nullptr);
        std::tm system_time = *localtime(&now);
        std::stringstream time;
        time << std::put_time(&system_time, "%Y%m%dT%H_%M_%S");
        return time.str();
    }

    DataLogging::DataLogging(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        /// This receives every DataLog message as a Sync operation (one at a time) and writes it to the file
        on<Trigger<DataLog>, Sync<DataLog>>().then([this](const DataLog& data) {
            // NBS File Format
            // Name      | Type               |  Description
            // ------------------------------------------------------------
            // header    | char[3]            | NBS packet header ☢ { 0xE2, 0x98, 0xA2 }
            // length    | uint32_t           | Length of this packet after this value
            // timestamp | uint64_t           | Timestamp the data was emitted in microseconds
            // hash      | uint64_t           | the 64bit hash for the payload type
            // payload   | char[length - 16]  | the data payload

            // Convert the timestamp to a 64bit microsecond timestamp
            uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(
                                        data.timestamp.time_since_epoch())
                                        .count();

            // The size of our output timestamp hash and data
            uint32_t size = data.data.size() + sizeof(data.hash) + sizeof(timestamp_us);

            // If the file isn't open, or writing this message will exceed our max size, make a new file
            if (!encoder || !encoder->is_open()
                || (encoder->get_bytes_written() + size + 3 + sizeof(timestamp_us) + sizeof(data.hash))
                       >= config.output.split_size) {
                if (encoder) {
                    encoder->close();
                }

                std::filesystem::path temp;

                if (!output_file_path.empty()) {
                    temp = output_file_path;
                    // Remove the first character of the filename
                    temp.replace_filename(
                        output_file_path.filename().string().substr(1,
                                                                    output_file_path.filename().string().size() - 1));
                    std::filesystem::rename(output_file_path, temp);
                }

                if (!index_file_path.empty()) {
                    temp = index_file_path;
                    // Remove the first character of the filename
                    temp.replace_filename(
                        index_file_path.filename().string().substr(1, index_file_path.filename().string().size() - 1));
                    std::filesystem::rename(index_file_path, temp);
                }

                // Creates directory for output
                std::filesystem::create_directories(config.output.directory / config.output.binary);

                // Creates the output ".nbs" file path.
                std::string ftime = formatted_time();
                output_file_path  = config.output.directory / config.output.binary / ("_" + ftime + ".nbs");

                // Creates the output ".idx" file path.
                index_file_path = output_file_path;
                index_file_path += ".idx";

                encoder = std::make_unique<utility::nbs::Encoder>(output_file_path, index_file_path);
            }

            encoder->write(data.timestamp, data.message_timestamp, data.hash, data.id, data.data);
        });

        on<Shutdown>().then([this] {
            encoder->close();

            std::filesystem::path temp;

            if (!output_file_path.empty()) {
                temp = output_file_path;
                // Remove the first character of the filename
                temp.replace_filename(
                    output_file_path.filename().string().substr(1, output_file_path.filename().string().size() - 1));
                std::filesystem::rename(output_file_path, temp);
            }

            if (!index_file_path.empty()) {
                temp = index_file_path;
                // Remove the first character of the filename
                temp.replace_filename(
                    index_file_path.filename().string().substr(1, index_file_path.filename().string().size() - 1));
                std::filesystem::rename(index_file_path, temp);
            }
        });

        on<Configuration, Trigger<CommandLineArguments>, Sync<DataLog>>("DataLogging.yaml")
            .then([this](const Configuration& cfg, const CommandLineArguments& argv) {
                log_level = cfg["log_level"].as<NUClear::LogLevel>();

                // Get the details we need to generate a log file name
                config.output.directory  = cfg["output"]["directory"].as<std::string>();
                config.output.split_size = cfg["output"]["split_size"].as<Expression>();

                // Get the name of the currently running binary
                std::vector<uint8_t> data(argv[0].cbegin(), argv[0].cend());
                data.push_back('\0');
                const auto* base     = basename(reinterpret_cast<const char*>(data.data()));
                config.output.binary = std::string(base);

                // Rescue any existing recorders that we want to keep
                std::map<std::string, ReactionHandle> new_handles;
                for (const auto& setting : cfg["messages"].config) {
                    auto name    = setting.first.as<std::string>();
                    bool enabled = setting.second.as<bool>();

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
                for (const auto& setting : cfg["messages"].config) {
                    auto name    = setting.first.as<std::string>();
                    bool enabled = setting.second.as<bool>();

                    // If we are enabling this, and it wasn't already enabled, enable it
                    if (new_handles.count(name) == 0 && enabled) {
                        log<NUClear::INFO>("Data logging for type", name, "enabled");
                        new_handles.insert(std::make_pair(name, activate_recorder(name)));
                    }
                }

                // New handles become the handles
                handles = std::move(new_handles);
            });
    }

}  // namespace module::support::logging
