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

DataLogging::DataLogging(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), bytes_written(0) {

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
        uint64_t timestamp_us =
            std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(data.timestamp.time_since_epoch())
                .count();

        // The size of our output timestamp hash and data
        uint32_t size = data.data.size() + sizeof(data.hash) + sizeof(timestamp_us);

        // If the file isn't open, or writing this message will exceed our max size, make a new file
        if (!output_file.is_open()
            || (bytes_written + size + 3 + sizeof(timestamp_us) + sizeof(data.hash)) >= config.output.split_size) {
            output_file.close();

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

            // Creates directory for output
            std::filesystem::create_directories(config.output.directory / config.output.binary);

            // Creates the output ".nbs" file.
            output_file_path = std::filesystem::path();
            output_file_path += config.output.directory / config.output.binary / ("_" + formatted_time() + ".nbs");
            output_file = std::ofstream(output_file_path);

            // Creates the output ".idx" file.
            index_file_path = output_file_path;
            index_file_path += ".idx";
            index_file = std::make_unique<zstr::ofstream>(index_file_path);

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

        // NBS Index File Format
        // Name      | Type               |  Description
        // ------------------------------------------------------------
        // hash      | uint64_t           | the 64bit hash for the payload type
        // id        | uint32_t           | the id field of the payload (or 0 if it does not have one)
        // timestamp | uint64_t           | Timestamp of the message or the emit timestamp in nanoseconds
        // offset    | uint64_t           | offset to start of radiation symbol ☢
        // size      | uint32_t           | Size of the whole packet from the radiation symbol

        uint32_t full_size = size + 7;

        index_file->write(reinterpret_cast<const char*>(&data.hash), sizeof(data.hash));
        index_file->write(reinterpret_cast<const char*>(&data.id), sizeof(data.id));
        index_file->write(reinterpret_cast<const char*>(&data.message_timestamp), sizeof(data.message_timestamp));
        index_file->write(reinterpret_cast<const char*>(&bytes_written), sizeof(bytes_written));
        index_file->write(reinterpret_cast<const char*>(&full_size), sizeof(full_size));
        index_file->flush();

        // Update the number of bytes we have written to the nbs file
        bytes_written += size + 3 + sizeof(timestamp_us) + sizeof(data.hash);
    });

    on<Shutdown>().then([this] {
        output_file.close();

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
            }
        });
}

}  // namespace module::support::logging
