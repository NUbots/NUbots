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

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nuclear>
#include <string>
#include <system_error>

#include "FileWatch.h"

#include "message/extension/Script.h"

#include "utility/file/fileutil.h"
#include "utility/input/ServoID.h"
#include "utility/strutil/strutil.h"
#include "utility/support/system.h"

namespace extension {

struct Script {
    std::string filename, hostname, platform;
    message::extension::Script script;
    YAML::Node config;

    Script(const std::string& filename,
           const std::string& hostname,
           const std::string& platform,
           const YAML::Node& config,
           const message::extension::Script& script)
        : filename(filename), hostname(hostname), platform(platform), script(script), config(config) {}

    Script(const std::string& filename, const std::string& hostname, const std::string& platform)
        : filename(filename), hostname(hostname), platform(platform), script(), config() {

        // Per robot scripts:    Scripts that are specific to a certain robot (e.g. darwin1).
        //                       These are to account for minor hardware variations in a robot and, as such, take
        //                       precedence over per platform scripts.
        // Per platform scripts: Scripts that are specific to a certain platform (e.g. darwin, igus, etc).
        //                       These are the default scripts, it is an error for this version of the script to not
        //                       exist.

        if (utility::file::exists(fmt::format("scripts/{}/{}", hostname, filename))) {
            NUClear::log<NUClear::INFO>("Parsing robot specific script:", filename);
            config = YAML::LoadFile(fmt::format("scripts/{}/{}", hostname, filename));
        }

        else if (utility::file::exists(fmt::format("scripts/{}/{}", platform, filename))) {
            NUClear::log<NUClear::INFO>("Parsing default platform script:", filename);
            config = YAML::LoadFile(fmt::format("scripts/{}/{}", platform, filename));
        }

        script = config.as<message::extension::Script>();
    }

    Script operator[](const std::string& key) {
        return Script(filename, hostname, platform, config[key], script);
    }

    const Script operator[](const std::string& key) const {
        return Script(filename, hostname, platform, config[key], script);
    }

    Script operator[](const char* key) {
        return Script(filename, hostname, platform, config[key], script);
    }

    const Script operator[](const char* key) const {
        return Script(filename, hostname, platform, config[key], script);
    }

    Script operator[](size_t index) {
        return Script(filename, hostname, platform, config[index], script);
    }

    const Script operator[](size_t index) const {
        return Script(filename, hostname, platform, config[index], script);
    }

    Script operator[](int index) {
        return Script(filename, hostname, platform, config[index], script);
    }

    const Script operator[](int index) const {
        return Script(filename, hostname, platform, config[index], script);
    }

    template <typename T>
    T as() const {
        return config.as<T>();
    }

    // All of these disables for this template are because the std::string constructor is magic and screwy
    template <
        typename T,
        typename Decayed = typename std::decay<T>::type,
        typename         = typename std::enable_if<
            !std::is_same<const char*, Decayed>::value && !std::is_same<std::allocator<char>, Decayed>::value
            && !std::is_same<std::initializer_list<char>, Decayed>::value && !std::is_same<char, Decayed>::value>::type>
    operator T() const {
        return config.as<T>();
    }

    // The conversion for string is fully specialised because strings get screwy
    // because of their auto conversion to const char* etc
    operator std::string() const {
        return config.as<std::string>();
    }
};
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

                std::string hostname(utility::support::getHostname());
                std::string platform(utility::support::getPlatform(hostname));

                // Set paths to the script files.
                auto robot_script    = fmt::format("scripts/{}/{}", hostname, path);
                auto platform_script = fmt::format("scripts/{}/{}", platform, path);

                // The platform script is the default script. This must exist!
                if (!utility::file::exists(platform_script)) {
                    throw std::runtime_error(fmt::format("Script file '{}' does not exist.", platform_script));
                }

                // Bind our default path
                DSLProxy<::extension::FileWatch>::bind<DSL>(reaction, platform_script, flags);

                // Bind our robot specific path if it exists
                if (utility::file::exists(robot_script)) {
                    DSLProxy<::extension::FileWatch>::bind<DSL>(reaction, robot_script, flags);
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
                        std::string hostname(utility::support::getHostname());
                        std::string platform(utility::support::getPlatform(hostname));

                        // Get relative path to script file.
                        auto components = utility::strutil::split(watch.path, '/');
                        std::string relative_path("");
                        bool flag = false;

                        for (const auto& component : components) {
                            // Ignore the hostname/platform name if they are present.
                            if (flag && (component.compare(hostname) != 0) && (component.compare(platform) != 0)) {
                                relative_path.append(component + "/");
                            }

                            // We want out paths relative to the script folder.
                            if (component.compare("scripts") == 0) {
                                flag = true;
                            }
                        }

                        // There will be a trailing / character.
                        relative_path.pop_back();

                        return std::make_shared<::extension::Script>(relative_path, hostname, platform);
                    }
                    catch (const YAML::ParserException& e) {
                        throw std::runtime_error(fmt::format("{} {}", watch.path, std::string(e.what())));
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
template <>
struct convert<message::extension::Script::Frame::Target> {
    static inline Node encode(const message::extension::Script::Frame::Target& rhs) {
        Node node;

        node["id"]       = static_cast<std::string>(utility::input::ServoID(rhs.id));
        node["position"] = rhs.position;
        node["gain"]     = rhs.gain;
        node["torque"]   = rhs.torque;

        return node;
    }

    static inline bool decode(const Node& node, message::extension::Script::Frame::Target& rhs) {
        try {
            rhs.id       = utility::input::ServoID(node["id"].as<std::string>());
            rhs.position = node["position"].as<float>();
            rhs.gain     = node["gain"].as<float>();
            rhs.torque   = node["torque"] ? node["torque"].as<float>() : 100.0f;
        }
        catch (const YAML::Exception& e) {
            NUClear::log<NUClear::ERROR>("Error parsing script -",
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
struct convert<message::extension::Script::Frame> {
    static inline Node encode(const message::extension::Script::Frame& rhs) {
        Node node;

        node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(rhs.duration).count();
        node["targets"]  = rhs.targets;

        return node;
    }

    static inline bool decode(const Node& node, message::extension::Script::Frame& rhs) {
        try {
            int millis = node["duration"].as<int>();
            std::chrono::milliseconds duration(millis);

            std::vector<message::extension::Script::Frame::Target> targets =
                node["targets"].as<std::vector<message::extension::Script::Frame::Target>>();
            rhs.duration = duration;
            rhs.targets  = std::move(targets);
        }
        catch (const YAML::Exception& e) {
            NUClear::log<NUClear::ERROR>("Error parsing script -",
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
struct convert<message::extension::Script> {
    static inline Node encode(const message::extension::Script& rhs) {
        Node node;

        node = rhs.frames;

        return node;
    }

    static inline bool decode(const Node& node, message::extension::Script& rhs) {
        try {
            std::vector<message::extension::Script::Frame> frames =
                node.as<std::vector<message::extension::Script::Frame>>();
            rhs.frames = std::move(frames);
        }
        catch (const YAML::Exception& e) {
            NUClear::log<NUClear::ERROR>("Error parsing script -",
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
