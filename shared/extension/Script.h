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
#include "utility/file/fileutil.h"
#include "utility/input/ServoID.h"
#include "utility/strutil/strutil.h"

namespace extension {

using ServoID = utility::input::ServoID;

/**
 * TODO document
 *
 * @author Alex Biddulph
 */
struct Script {
    struct Frame {
        struct Target {
            Target() : id(), position(0.0f), gain(0.0f), torque(0.0f) {}
            Target(const ServoID& servo, float pos, float gain, float torque)
                : id(servo), position(pos), gain(gain), torque(torque) {}
            Target(const Target& other)
                : id(other.id), position(other.position), gain(other.gain), torque(other.torque) {}
            Target(Target&& other)
                : id(std::move(other.id))
                , position(std::move(other.position))
                , gain(std::move(other.gain))
                , torque(std::move(other.torque)) {}
            Target& operator=(const Target& other) {
                id       = other.id;
                position = other.position;
                gain     = other.gain;
                torque   = other.torque;
                return *this;
            }
            Target& operator=(Target&& other) {
                id       = std::move(other.id);
                position = std::move(other.position);
                gain     = std::move(other.gain);
                torque   = std::move(other.torque);
                return *this;
            }

            ServoID id;
            float position;
            float gain;
            float torque;
        };

        Frame() : duration(), targets() {}
        Frame(const NUClear::clock::duration& dur, const std::vector<Target>& targets)
            : duration(dur), targets(targets) {}

        NUClear::clock::duration duration;
        std::vector<Target> targets;
    };

    std::string fileName, hostname, platform;
    YAML::Node config;
    std::vector<Frame> frames;

    Script()
        : fileName(), hostname(Script::getHostname()), platform(Script::getPlatform(hostname)), config(), frames() {}

    Script(const std::vector<Frame>& frames)
        : fileName()
        , hostname(Script::getHostname())
        , platform(Script::getPlatform(hostname))
        , config()
        , frames(frames) {}

    Script(const std::string& fileName,
           const std::string& hostname,
           const std::string& platform,
           const YAML::Node& config,
           const std::vector<Frame>& frames)
        : fileName(fileName), hostname(hostname), platform(platform), config(config), frames(frames) {}

    Script(const std::string& fileName, const std::string& hostname, const std::string& platform)
        : fileName(fileName), hostname(hostname), platform(platform), config(), frames() {

        // Per robot scripts:    Scripts that are specific to a certain robot (e.g. darwin1).
        //                       These are to account for minor hardware variations in a robot and, as such, take
        //                       precedence over per platform scripts.
        // Per platform scripts: Scripts that are specific to a certain platform (e.g. darwin, igus, etc).
        //                       These are the default scripts, it is an error for this version of the script to not
        //                       exist.

        if (utility::file::exists("scripts/" + hostname + "/" + fileName)) {
            NUClear::log<NUClear::INFO>("Parsing robot specific script:", fileName);
            config = YAML::LoadFile("scripts/" + hostname + "/" + fileName);
        }

        else if (utility::file::exists("scripts/" + platform + "/" + fileName)) {
            NUClear::log<NUClear::INFO>("Parsing default platform script:", fileName);
            config = YAML::LoadFile("scripts/" + platform + "/" + fileName);
        }

        frames = config.as<std::vector<Frame>>();
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

    Script operator[](const std::string& key) {
        return Script(fileName, hostname, platform, config[key], frames);
    }

    const Script operator[](const std::string& key) const {
        return Script(fileName, hostname, platform, config[key], frames);
    }

    Script operator[](const char* key) {
        return Script(fileName, hostname, platform, config[key], frames);
    }

    const Script operator[](const char* key) const {
        return Script(fileName, hostname, platform, config[key], frames);
    }

    Script operator[](size_t index) {
        return Script(fileName, hostname, platform, config[index], frames);
    }

    const Script operator[](size_t index) const {
        return Script(fileName, hostname, platform, config[index], frames);
    }

    Script operator[](int index) {
        return Script(fileName, hostname, platform, config[index], frames);
    }

    const Script operator[](int index) const {
        return Script(fileName, hostname, platform, config[index], frames);
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

    void save(const std::string& file) {
        std::ofstream yaml("scripts/" + hostname + "/" + file, std::ios::trunc | std::ios::out);
        yaml << YAML::convert<std::vector<Frame>>::encode(frames);
        yaml.close();
    }
};

/**
 * TODO document
 *
 * @author Trent Houliston
 */
struct ExecuteScriptByName {
    ExecuteScriptByName(const size_t& id,
                        const std::string& script,
                        const NUClear::clock::time_point& start = NUClear::clock::now())
        : sourceId(id), scripts(1, script), duration_modifier(1, 1.0), start(start){};
    ExecuteScriptByName(const size_t& id,
                        const std::string& script,
                        const double& duration_mod,
                        const NUClear::clock::time_point& start = NUClear::clock::now())
        : sourceId(id), scripts(1, script), duration_modifier(1, duration_mod), start(start){};
    ExecuteScriptByName(const size_t& id,
                        const std::vector<std::string>& scripts,
                        const NUClear::clock::time_point& start = NUClear::clock::now())
        : sourceId(id), scripts(scripts), duration_modifier(scripts.size(), 1.0), start(start){};
    ExecuteScriptByName(const size_t& id,
                        const std::vector<std::string>& scripts,
                        const std::vector<double>& duration_mod,
                        const NUClear::clock::time_point& start = NUClear::clock::now())
        : sourceId(id), scripts(scripts), duration_modifier(duration_mod), start(start) {
        while (scripts.size() > duration_modifier.size()) {
            duration_modifier.push_back(1.0);
        }
        while (scripts.size() < duration_modifier.size()) {
            duration_modifier.pop_back();
        }
    };
    size_t sourceId;
    std::vector<std::string> scripts;
    std::vector<double> duration_modifier;
    NUClear::clock::time_point start;
};

/**
 * TODO document
 *
 * @author Trent Houliston
 */
struct ExecuteScript {
    ExecuteScript(const size_t& id, const Script& script, NUClear::clock::time_point start = NUClear::clock::now())
        : sourceId(id), scripts(1, script), duration_modifier(1, 1.0), start(start){};
    ExecuteScript(const size_t& id,
                  const Script& script,
                  double duration_mod              = 1.0,
                  NUClear::clock::time_point start = NUClear::clock::now())
        : sourceId(id), scripts(1, script), duration_modifier(1, duration_mod), start(start){};
    ExecuteScript(const size_t& id,
                  const std::vector<Script>& scripts,
                  NUClear::clock::time_point start = NUClear::clock::now())
        : sourceId(id), scripts(scripts), duration_modifier(scripts.size(), 1.0), start(start){};
    ExecuteScript(const size_t& id,
                  const std::vector<Script>& scripts,
                  const std::vector<double>& duration_mod,
                  NUClear::clock::time_point start = NUClear::clock::now())
        : sourceId(id), scripts(scripts), duration_modifier(duration_mod), start(start) {
        while (scripts.size() > duration_modifier.size()) {
            duration_modifier.push_back(1.0);
        }
        while (scripts.size() < duration_modifier.size()) {
            duration_modifier.pop_back();
        }
    };
    size_t sourceId;
    std::vector<Script> scripts;
    std::vector<double> duration_modifier;
    NUClear::clock::time_point start;
};

inline Script operator+(const Script& s1, const Script& s2) {
    Script s;
    s.frames.insert(s.frames.end(), s1.frames.begin(), s1.frames.end());
    s.frames.insert(s.frames.end(), s2.frames.begin(), s2.frames.end());
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

                std::string hostname(::extension::Script::getHostname()),
                    platform(::extension::Script::getPlatform(hostname));

                // Set paths to the script files.
                auto robotScript    = "scripts/" + hostname + "/" + path;
                auto platformScript = "scripts/" + platform + "/" + path;

                // The platform script is the default script. This must exist!
                if (!utility::file::exists(platformScript)) {
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
                        std::string hostname(::extension::Script::getHostname()),
                            platform(::extension::Script::getPlatform(hostname));

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
template <>
struct convert<::extension::Script::Frame::Target> {
    static inline Node encode(const ::extension::Script::Frame::Target& rhs) {
        Node node;

        node["id"]       = static_cast<std::string>(rhs.id);
        node["position"] = rhs.position;
        node["gain"]     = rhs.gain;
        node["torque"]   = rhs.torque;

        return node;
    }

    static inline bool decode(const Node& node, ::extension::Script::Frame::Target& rhs) {
        try {
            rhs = {node["id"].as<std::string>(),
                   node["position"].as<float>(),
                   node["gain"].as<float>(),
                   node["torque"] ? node["torque"].as<float>() : 100};
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
struct convert<::extension::Script::Frame> {
    static inline Node encode(const ::extension::Script::Frame& rhs) {
        Node node;

        node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(rhs.duration).count();
        node["targets"]  = rhs.targets;

        return node;
    }

    static inline bool decode(const Node& node, ::extension::Script::Frame& rhs) {
        try {
            int millis = node["duration"].as<int>();
            std::chrono::milliseconds duration(millis);

            std::vector<::extension::Script::Frame::Target> targets =
                node["targets"].as<std::vector<::extension::Script::Frame::Target>>();
            rhs = {duration, std::move(targets)};
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
struct convert<::extension::Script> {
    static inline Node encode(const ::extension::Script& rhs) {
        Node node;

        node = rhs.frames;

        return node;
    }

    static inline bool decode(const Node& node, ::extension::Script& rhs) {
        try {
            std::vector<::extension::Script::Frame> frames = node.as<std::vector<::extension::Script::Frame>>();
            rhs                                            = {std::move(frames)};
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
