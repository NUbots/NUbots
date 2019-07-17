#!/usr/bin/env python3

import sys
import os
import pkgutil
from textwrap import dedent
import google.protobuf.message

if __name__ == "__main__":
    shared_folder = sys.argv[1]
    cpp_file = sys.argv[2]
    yaml_file = sys.argv[3]

    # Load all our protocol buffer files as modules into this file
    includes = []
    sys.path.append(shared_folder)
    for dir_name, subdir, files in os.walk(shared_folder):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith("pb2"):

                # Work out what header file this came from
                include = os.path.join(os.path.relpath(dir_name, shared_folder), "{}.h".format(module_name[:-4]))

                # If it's one of ours include it
                if include.startswith("message"):
                    includes.append(include)

                # Load our protobuf module
                loader.find_module(module_name).load_module(module_name)

    # Now that we've imported them all get all the subclasses of protobuf message
    messages = set()
    for message in google.protobuf.message.Message.__subclasses__():

        # Work out our original protobuf type
        pb_type = ".".join(message.DESCRIPTOR.full_name.split(".")[1:])

        # Only include our own messages
        if pb_type.startswith("message.") and not message.DESCRIPTOR.GetOptions().map_entry:
            messages.add(pb_type)

    messages = list(messages)

    # The base of our source file we will be filling in
    source = dedent(
        """\
        #include "DataLogging.h"
        #include <iomanip>

        #include "extension/Configuration.h"
        #include "utility/file/fileutil.h"

        {includes}

        namespace module {{
        namespace support {{
            namespace logging {{

                using extension::Configuration;

                DataLogging::DataLogging(std::unique_ptr<NUClear::Environment> environment)
                    : Reactor(std::move(environment)), fd(-1), output_file(), handles() {{

                    on<Trigger<DataLog>, Sync<DataLog>>().then([this](const DataLog& data) {{

                        // FILE FORMAT
                        // TYPE       DATA
                        // char[3]    RADIATION SYMBOL {{ 0xE2, 0x98, 0xA2 }}
                        // uint32_t   SIZE OF NEXT PACKET
                        // uint64_t   TIMESTAMP DATA WAS EMITTED IN MICROSECONDS
                        // uint64_t   DATA TYPE HASH

                        // If the file isn't open skip
                        if (!output_file.is_open()) {{
                            return;
                        }}

                        using namespace std::chrono;

                        uint64_t timestamp_us =
                            duration_cast<duration<uint64_t, std::micro>>(data.timestamp.time_since_epoch()).count();

                        // The size of our output timestamp hash and data
                        uint32_t size = data.data.size() + sizeof(data.hash) + sizeof(timestamp_us);

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

                        // Write the acutal packet data
                        output_file.write(data.data.data(), data.data.size());
                        output_file.flush();
                    }});

                    // Register all our recording handles
        {record_handles}

                    on<Configuration, Trigger<NUClear::message::CommandLineArguments>, Sync<DataLog>>("DataLogging.yaml")
                        .then([this](const Configuration& config, const NUClear::message::CommandLineArguments& argv) {{

                            // Enable the streams we are after and if none are enabled, we don't have to log
                            bool logging = false;
                            for (auto& setting : config["messages"].config) {{
                                // Get the name of the type
                                std::string name = setting.first.as<std::string>();
                                bool enabled     = setting.second.as<bool>();

                                if (handles.find(name) != handles.end()) {{
                                    auto& handle = handles[name];

                                    // We found something to log, start logging!
                                    logging |= enabled;

                                    if (enabled && !handle.enabled()) {{
                                        handle.enable();
                                        log<NUClear::INFO>("Data logging for type", name, "enabled");
                                    }}
                                    else if (!enabled && handle.enabled()) {{
                                        handle.disable();
                                        log<NUClear::INFO>("Data logging for type", name, "disabled");
                                    }}
                                }}
                                else {{
                                    log<NUClear::WARN>("This system does not know about the message type", name);
                                }}
                            }}

                            // If we should start logging and we are not already
                            // Or if we are logging and our logging directory changed
                            if ((logging && !output_file.is_open())
                                || (logging && output_file.is_open() && log_dir != config["directory"].as<std::string>())) {{
                                // Close our existing file if it is open
                                if (output_file.is_open()) {{
                                    output_file.close();
                                }}

                                log_dir = config["directory"].as<std::string>();

                                // Make the time into a folder pattern
                                std::time_t now    = time(0);
                                std::tm systemTime = *localtime(&now);
                                std::stringstream logfile;

                                // Get the name of the currently running binary
                                std::vector<char> data(argv[0].cbegin(), argv[0].cend());
                                data.push_back('\\0');
                                const auto* base = basename(data.data());
                                std::string base_str(base);

                                // Create a directory in our output folder
                                utility::file::makeDirectory(log_dir, true);

                                // Create a directory for our binary name
                                utility::file::makeDirectory(std::string(log_dir) + "/" + base_str, true);

                                // Create our log file full output path
                                logfile << log_dir << "/" << base_str << "/" << std::put_time(&systemTime, "%Y%m%dT%H_%M_%S")
                                        << ".nbs";

                                log<NUClear::INFO>("Logging to", logfile.str());

                                // Open the file
                                output_file = std::ofstream(logfile.str());
                            }}
                        }});
                }}
            }}  // namespace logging
        }}  // namespace support
        }}  // namespace module\n"""
    )

    # Work out our includes
    includes = ['#include "{}"'.format(i) for i in includes]

    # Make our recording handles
    handle_template = '            handles["{0}"] = on<Trigger<{1}>>().then([this](const {1}& d) {{ emit(log_encode(d)); }}).disable();'
    handles = [handle_template.format(m, m.replace(".", "::")) for m in sorted(messages)]

    with open(cpp_file, "w") as f:
        f.write(source.format(includes="\n".join(includes), record_handles="\n".join(handles)))

    # Now generate our yaml file
    yaml_template = dedent(
        """\
        directory: log

        messages:
        {messages}
    """
    )

    yaml_keys = ["  {}: false".format(m) for m in sorted(messages)]

    # and write it out
    with open(yaml_file, "w") as f:
        f.write(yaml_template.format(messages="\n".join(yaml_keys)))
