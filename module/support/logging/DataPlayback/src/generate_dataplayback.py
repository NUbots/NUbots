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
        #include "DataPlayback.h"

        #include "extension/Configuration.h"
        #include "read_packet.h"

        {includes}

        namespace module {{
        namespace support {{
            namespace logging {{

                using extension::Configuration;

                DataPlayback::DataPlayback(std::unique_ptr<NUClear::Environment> environment)
                    : Reactor(std::move(environment)), first_timecode(0) {{

        {players}

                    // Loop two times per second to check we are buffered enough
                    playback_handle = on<Every<2, Per<std::chrono::seconds>>, Sync<DataPlayback>, Single>().then([this] {{

                        bool buffered = false;
                        while (!buffered) {{

                            try {{
                                // Read our next packet from the stream
                                Packet p = read_packet(input_file);

                                // If our first_timecode is 0, this is our first packet for this cycle
                                if (first_timecode.count() == 0) {{
                                    start_time     = NUClear::clock::now();
                                    first_timecode = p.timecode;
                                }}

                                // Work out when we should emit this packet
                                auto emit_time = start_time + p.timecode - first_timecode;

                                // Check if we care about this packet
                                if (players.find(p.hash) != players.end() && players[p.hash].enabled) {{

                                    // Emit this packet at the appropriate time
                                    players[p.hash].emit(emit_time, p.payload);
                                }}

                                // If this time is further in the future than our buffer time we can finish for this round
                                buffered = emit_time - NUClear::clock::now() > buffer_time;
                            }}

                            catch (const std::exception& ex) {{

                                // If we reached the end of the file we either terminate or loop depending on the setting
                                if (input_file.eof()) {{

                                    log<NUClear::INFO>("Playback of file", current_file, "finished");

                                    // If we are looping go back to the start of the file and reset our timecode
                                    if (loop_playback) {{
                                        input_file.clear();
                                        input_file.seekg(0);
                                        first_timecode = std::chrono::microseconds(0);

                                        log<NUClear::INFO>("Restarting playback of file", current_file);
                                    }}
                                    // Otherwise disable this reaction and stop
                                    else {{
                                        input_file.clear();
                                        playback_handle.disable();
                                        buffered = true;
                                    }}
                                }}
                                // We tried to read something funny (possibly tried to read too many bytes)
                                else if (input_file.bad() || input_file.fail()) {{
                                    input_file.clear();
                                }}
                                else {{
                                    // We don't know what's up, throw the exception again
                                    throw ex;
                                }}
                            }}
                        }}
                    }});
                    playback_handle.disable();

                    on<Configuration, Sync<DataPlayback>>("DataPlayback.yaml").then([this](const Configuration& config) {{

                        // Update which types we will be playing
                        for (auto& setting : config["messages"].config) {{
                            // Get the name of the type
                            std::string name = setting.first.as<std::string>();
                            // Hash our type to work out our type on the wire
                            uint64_t hash = XXH64(name.c_str(), name.size(), 0x4e55436c);
                            bool enabled  = setting.second.as<bool>();

                            // Message if we have enabled/disabled a particular message type
                            if (players.find(hash) != players.end()) {{
                                auto& player = players[hash];

                                if (enabled && !player.enabled) {{
                                    player.enabled = true;
                                    log<NUClear::INFO>("Playback for message type", name, "enabled");
                                }}
                                else if (!enabled && player.enabled) {{
                                    player.enabled = false;
                                    log<NUClear::INFO>("Playback for message type", name, "disabled");
                                }}
                            }}
                            else {{
                                log<NUClear::WARN>("The playback system does not know about the message type", name);
                            }}
                        }}

                        // Get our other config parameters
                        auto file     = config["file"].as<std::string>();
                        loop_playback = config["loop_playback"].as<bool>();
                        buffer_time   = std::chrono::milliseconds(config["buffer_time"].as<uint64_t>());

                        // If we are changing files to no file
                        if (file != current_file && file == "") {{

                            log<NUClear::WARN>("Stopping playback of file", current_file);

                            current_file = file;

                            // Disable the callback so we stop buffering
                            playback_handle.enable();

                            // Close the file
                            if (input_file.is_open()) {{
                                input_file.close();
                            }}
                        }}
                        // We are playing a new file
                        else if (file != current_file) {{

                            if (input_file.is_open()) {{
                                log<NUClear::WARN>("Stopping playback of file", current_file);
                                input_file.close();
                            }}

                            current_file = file;

                            log<NUClear::INFO>("Starting playback of file", current_file);

                            first_timecode = std::chrono::microseconds(0);
                            input_file.open(file);
                            playback_handle.enable();
                        }}
                    }});
                }}

            }}  // namespace logging
        }}  // namespace support
        }}  // namespace module

        """
    )

    # Work out our includes and players
    includes = ['#include "{}"'.format(i) for i in includes]
    players = ["            add_player<{}>();".format(m.replace(".", "::")) for m in messages]

    with open(cpp_file, "w") as f:
        f.write(source.format(includes="\n".join(includes), players="\n".join(players)))

    # Now generate our yaml file
    yaml_template = dedent(
        """\
        file: ""
        loop_playback: true
        # The amount of time into the future to buffer, should be at least 1000ms
        buffer_time: 1500

        messages:
        {messages}
    """
    )

    yaml_keys = ["  {}: false".format(m) for m in sorted(messages)]

    # and write it out
    with open(yaml_file, "w") as f:
        f.write(yaml_template.format(messages="\n".join(yaml_keys)))
