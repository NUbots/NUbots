/*
 * Copyright (C) 2013-2016 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef EXTENSION_SCRIPT_H
#define EXTENSION_SCRIPT_H

#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <nuclear>
#include <regex>
#include <string>
#include <system_error>

#include "FileWatch.h"

#include "message/motion/script/Script.h"
#include "utility/file/fileutil.h"
#include "utility/input/ServoID.h"
#include "utility/strutil/strutil.h"

namespace extension {

using ServoID       = utility::input::ServoID;
using ScriptMessage = message::motion::script::Script;

/**
 * TODO document
 *
 * @author Alex Biddulph
 */
struct Script {
    std::string filename, hostname, platform;
    YAML::Node yaml;
    ScriptMessage message;

    Script()
        : filename(), hostname(Script::getHostname()), platform(Script::getPlatform(hostname)), yaml(), message() {}

    Script(const ScriptMessage& message)
        : filename()
        , hostname(Script::getHostname())
        , platform(Script::getPlatform(hostname))
        , yaml()
        , message(message) {}

    Script(const std::string& filename,
           const std::string& hostname,
           const std::string& platform,
           const YAML::Node& yaml,
           const ScriptMessage& message)
        : filename(filename), hostname(hostname), platform(platform), yaml(yaml), message(message) {}

    Script(const std::string& filename, const std::string& hostname, const std::string& platform)
        : filename(filename), hostname(hostname), platform(platform), yaml(), message() {

        // Per robot scripts:    Scripts that are specific to a certain robot (e.g. darwin1).
        //                       These are to account for minor hardware variations in a robot and, as such, take
        //                       precedence over per platform scripts.
        // Per platform scripts: Scripts that are specific to a certain platform (e.g. darwin, igus, etc).
        //                       These are the default scripts, it is an error for this version of the script to not
        //                       exist.

        if (utility::file::exists("scripts/" + hostname + "/" + filename)) {
            NUClear::log<NUClear::INFO>("Parsing robot specific script:", filename);
            yaml = YAML::LoadFile("scripts/" + hostname + "/" + filename);
        }

        else if (utility::file::exists("scripts/" + platform + "/" + filename)) {
            NUClear::log<NUClear::INFO>("Parsing default platform script:", filename);
            yaml = YAML::LoadFile("scripts/" + platform + "/" + filename);
        }

        message          = ScriptMessage();
        message.filename = filename;
        message.servos   = yaml.as<std::vector<ScriptMessage::Servo>>();
    }

    static inline std::string getHostname() {
        // Get hostname so we can find the correct per-robot script directory.
        char host[255];
        gethostname(host, 255);
        return host;
    }

    static inline std::string getPlatform(const std::string& hostname) {
        // It is assumed that all hostnames are in the format <platform name><robot number>,
        // such that the regular expression
        // [a-z]+[0-9]+?
        // will match all hostnames
        std::regex re("([a-z]+)([0-9]+)?");
        std::smatch match;

        if (std::regex_match(hostname, match, re)) {
            // match[0] will be the full string
            // match[1] the first match (platform name)
            // match[2] the second match (robot number)
            return match[1].str();
        }

        else {
            throw std::system_error(
                -1, std::system_category(), ("Failed to extract platform name from '" + hostname + "'."));
        }
    }

    void save(const std::string& file) {
        std::ofstream yamlFile("scripts/" + hostname + "/" + file, std::ios::trunc | std::ios::out);
        yamlFile << YAML::convert<std::vector<ScriptMessage::Servo>>::encode(message.servos);
        yamlFile.close();
    }
};

inline Script operator+(const Script& s1, const Script& s2) {
    Script s;
    s.message.servos.insert(s.message.servos.end(), s1.message.servos.begin(), s1.message.servos.end());
    s.message.servos.insert(s.message.servos.end(), s2.message.servos.begin(), s2.message.servos.end());
    return s;
}

}  // namespace extension

// NUClear configuration extension
namespace NUClear {
namespace dsl {
    namespace operation {
        template <>
        struct DSLProxy<::extension::Script> {
            template <typename DSL>
            static inline void bind(const std::shared_ptr<threading::Reaction>& reaction, const std::string& path) {
                auto flags = ::extension::FileWatch::ATTRIBUTE_MODIFIED | ::extension::FileWatch::CREATED
                             | ::extension::FileWatch::UPDATED | ::extension::FileWatch::MOVED_TO;

                std::string hostname(extension::Script::getHostname());
                std::string platform(extension::Script::getPlatform(hostname));

                // Set paths to the script files.
                auto robotScript    = "scripts/" + hostname + "/" + path;
                auto platformScript = "scripts/" + platform + "/" + path;

                // The platform script is the default script. This must exist!
                if (!utility::file::exists(platformScript)) {
                    NUClear::log<NUClear::ERROR>("Failed to find script: ", platformScript);
                    throw std::runtime_error("Script file '" + platformScript + "' does not exist.");
                }

                // Bind our default path
                DSLProxy<::extension::FileWatch>::bind<DSL>(reaction, platformScript, flags);

                // Bind our robot specific path if it exists
                if (utility::file::exists(robotScript)) {
                    DSLProxy<::extension::FileWatch>::bind<DSL>(reaction, robotScript, flags);
                }
            }

            template <typename DSL>
            static inline std::shared_ptr<::extension::Script> get(threading::Reaction& t) {

                // Get the file watch event
                ::extension::FileWatch watch = DSLProxy<::extension::FileWatch>::get<DSL>(t);

                // Check if the watch is valid
                if (watch && utility::strutil::endsWith(watch.path, ".yaml")) {
                    // Return our yaml file
                    try {
                        std::string hostname(extension::Script::getHostname()),
                            platform(extension::Script::getPlatform(hostname));

                        // Get relative path to script file.
                        auto components = utility::strutil::split(watch.path, '/');
                        std::string relativePath("");
                        bool flag = false;

                        for (const auto& component : components) {
                            // Ignore the hostname/platform name if they are present.
                            if (flag && (component.compare(hostname) != 0) && (component.compare(platform) != 0)) {
                                relativePath.append(component + "/");
                            }

                            // We want out paths relative to the script folder.
                            if (component.compare("scripts") == 0) {
                                flag = true;
                            }
                        }

                        // There will be a trailing / character.
                        relativePath.pop_back();

                        return std::make_shared<::extension::Script>(relativePath, hostname, platform);
                    }
                    catch (const YAML::ParserException& e) {
                        NUClear::log<NUClear::ERROR>("Error parsing script yaml file:",
                                                     watch.path,
                                                     "YAML ParserException:",
                                                     std::string(e.what()));
                        throw std::runtime_error(watch.path + " " + std::string(e.what()));
                    }
                }
                else {
                    // Return an empty configuration (which will show up invalid)
                    return std::shared_ptr<::extension::Script>(nullptr);
                }
            }
        };
    }  // namespace operation

    // Script is transient
    namespace trait {
        template <>
        struct is_transient<std::shared_ptr<::extension::Script>> : public std::true_type {};
    }  // namespace trait
}  // namespace dsl
}  // namespace NUClear

namespace YAML {
using ScriptMessage = message::motion::script::Script;

template <>
struct convert<ScriptMessage::Servo::Frame> {
    static inline Node encode(const ScriptMessage::Servo::Frame& rhs) {
        Node node;

        node["time"]   = std::chrono::duration_cast<std::chrono::milliseconds>(rhs.time).count();
        node["angle"]  = rhs.angle;
        node["p_gain"] = rhs.p_gain;
        node["i_gain"] = rhs.i_gain;
        node["d_gain"] = rhs.d_gain;
        node["torque"] = rhs.torque;

        return node;
    }

    static inline bool decode(const Node& node, ScriptMessage::Servo::Frame& rhs) {
        try {
            int millis = node["time"].as<int>();
            std::chrono::milliseconds _time(millis);

            rhs = {_time,
                   node["angle"].as<float>(),
                   node["p_gain"].as<float>(),
                   node["i_gain"].as<float>(),
                   node["d_gain"].as<float>(),
                   node["torque"] ? node["torque"].as<float>() : 100};
        }
        catch (const YAML::Exception& e) {
            NUClear::log<NUClear::ERROR>("Error decoding Script::Servo::Frame in script -",
                                         "Line:",
                                         e.mark.line,
                                         "Column:",
                                         e.mark.column,
                                         "Pos:",
                                         e.mark.pos,
                                         "Message:",
                                         e.msg);
            return false;
        }

        return true;
    }
};

template <>
struct convert<ScriptMessage::Servo> {
    static inline Node encode(const ScriptMessage::Servo& rhs) {
        Node node;

        node["id"]     = static_cast<std::string>(rhs.id);
        node["frames"] = rhs.frames;

        return node;
    }

    static inline bool decode(const Node& node, ScriptMessage::Servo& rhs) {
        try {
            std::vector<ScriptMessage::Servo::Frame> frames =
                node["frames"].as<std::vector<ScriptMessage::Servo::Frame>>();

            rhs = {node["id"].as<std::string>(), std::move(frames)};
        }
        catch (const YAML::Exception& e) {
            NUClear::log<NUClear::ERROR>("Error decoding Script::Servo in script -",
                                         "Line:",
                                         e.mark.line,
                                         "Column:",
                                         e.mark.column,
                                         "Pos:",
                                         e.mark.pos,
                                         "Message:",
                                         e.msg);
            return false;
        }

        return true;
    }
};

template <>
struct convert<ScriptMessage> {
    static inline Node encode(const ScriptMessage& rhs) {
        Node node;

        node = rhs.servos;

        return node;
    }

    static inline bool decode(const Node& node, ScriptMessage& rhs) {
        try {
            std::vector<ScriptMessage::Servo> servos = node.as<std::vector<ScriptMessage::Servo>>();
            rhs.servos                               = {std::move(servos)};
        }
        catch (const YAML::Exception& e) {
            NUClear::log<NUClear::ERROR>("Error decoding Script in script -",
                                         "Line:",
                                         e.mark.line,
                                         "Column:",
                                         e.mark.column,
                                         "Pos:",
                                         e.mark.pos,
                                         "Message:",
                                         e.msg);
            return false;
        }

        return true;
    }
};
}  // namespace YAML

#endif  // EXTENSION_SCRIPT_H
