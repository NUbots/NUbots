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

#include <cstdlib>
#include <nuclear>
#include <yaml-cpp/yaml.h>

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

        std::string fileName, hostname, binary;
        YAML::Node config;
        std::vector<Frame> frames;

        Script() : fileName(), hostname(), binary(), config(), frames()
        {
            // Get hostname so we can find the correct per-robot script directory.
            char host[255];
            gethostname(host, 255);
            hostname = host;

            // Get the command line arguments so we can find the current binary's name.
            std::shared_ptr<const NUClear::message::CommandLineArguments> args = NUClear::dsl::store::DataStore<NUClear::message::CommandLineArguments>::get();

            std::vector<char> data(args->at(0).cbegin(), args->at(0).cend());
            data.push_back('\0');
            binary = basename(data.data());
        }

        
        Script(const std::vector<Frame>& frames) : fileName(), hostname(), binary(), config(), frames(frames) 
        {
            // Get hostname so we can find the correct per-robot script directory.
            char host[255];
            gethostname(host, 255);
            hostname = host;

            // Get the command line arguments so we can find the current binary's name.
            std::shared_ptr<const NUClear::message::CommandLineArguments> args = NUClear::dsl::store::DataStore<NUClear::message::CommandLineArguments>::get();

            std::vector<char> data(args->at(0).cbegin(), args->at(0).cend());
            data.push_back('\0');
            binary = basename(data.data());
        }

        Script(const std::string& fileName, const std::string& hostname, const std::string& binary, const YAML::Node& config, const std::vector<Frame>& frames) 
            : fileName(fileName)
            , hostname(hostname)
            , binary(binary)
            , config(config)
            , frames(frames) {}

        Script(const std::string& fileName, const std::string& hostname, const std::string& binary) : fileName(fileName), hostname(hostname), binary(binary), config(), frames()
        {
            if (utility::file::exists("scripts/" + binary + "/" + fileName))
            {
                config = YAML::LoadFile("scripts/" + binary + "/" + fileName);
            }

            else if (utility::file::exists("scripts/" + hostname + "/" + fileName))
            {
                config = YAML::LoadFile("scripts/" + hostname + "/" + fileName);
            }

            else if (utility::file::exists("scripts/" + fileName))
            {
                config = YAML::LoadFile("scripts/" + fileName);
            }

            NUClear::log("Parsing script:", fileName);
            frames = config.as<std::vector<Frame>>();
        }

        Script operator [] (const std::string& key) {
            return Script(fileName, hostname, binary, config[key], frames);
        }

        const Script operator [] (const std::string& key) const {
            return Script(fileName, hostname, binary, config[key], frames);
        }

        Script operator [] (const char* key) {
            return Script(fileName, hostname, binary, config[key], frames);
        }

        const Script operator [] (const char* key) const {
            return Script(fileName, hostname, binary, config[key], frames);
        }

        Script operator [] (size_t index) {
            return Script(fileName, hostname, binary, config[index], frames);
        }

        const Script operator [] (size_t index) const {
            return Script(fileName, hostname, binary, config[index], frames);
        }

        Script operator [] (int index) {
            return Script(fileName, hostname, binary, config[index], frames);
        }

        const Script operator [] (int index) const {
            return Script(fileName, hostname, binary, config[index], frames);
        }

        template <typename T>
        T as() const {
            return config.as<T>();
        }

        // All of these disables for this template are because the std::string constructor is magic and screwy
        template <
                typename T
                , typename Decayed = typename std::decay<T>::type
                , typename = typename std::enable_if<
                        !std::is_same<
                                const char*
                                , Decayed
                        >::value
                        && !std::is_same<
                                std::allocator<char>
                                , Decayed
                        >::value
                        && !std::is_same<
                                std::initializer_list<char>
                                , Decayed
                        >::value
                        && !std::is_same<
                                char
                                , Decayed
                        >::value
                >::type
        >
        operator T() const {
            return config.as<T>();
        }

        // The conversion for string is fully specialised because strings get screwy
        // because of their auto conversion to const char* etc
        operator std::string() const {
            return config.as<std::string>();
        }

        void save(const std::string& file)
        {
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
        ExecuteScriptByName(const size_t& id, const std::string script, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts({script}), start(start) {};
        ExecuteScriptByName(const size_t& id, const std::vector<std::string> scripts, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts(scripts), start(start) {};
        size_t sourceId;
        std::vector<std::string> scripts;
        NUClear::clock::time_point start;
    };

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct ExecuteScript {
        ExecuteScript(const size_t& id, const Script& script, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts({script}), start(start) {};
        ExecuteScript(const size_t& id, const std::vector<Script>& scripts, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts(scripts), start(start) {};
        size_t sourceId;
        std::vector<Script> scripts;
        NUClear::clock::time_point start;
    };

    inline Script operator +(const Script& s1, const Script& s2){
        Script s;
        s.frames.insert(s.frames.end(), s1.frames.begin(), s1.frames.end());
        s.frames.insert(s.frames.end(), s2.frames.begin(), s2.frames.end());
        return s;
    } 

}  // extension

// NUClear configuration extension
namespace NUClear {
    namespace dsl {
        namespace operation {
            template <>
            struct DSLProxy<::extension::Script> {
                template <typename DSL, typename TFunc>
                static inline std::tuple<threading::ReactionHandle, threading::ReactionHandle, threading::ReactionHandle>
                    bind(Reactor& reactor, const std::string& label, TFunc&& callback, const std::string& path) {
                    auto flags = ::extension::FileWatch::ATTRIBUTE_MODIFIED
                               | ::extension::FileWatch::CREATED
                               | ::extension::FileWatch::UPDATED
                               | ::extension::FileWatch::MOVED_TO;

                    // Get hostname so we can find the correct per-robot script directory.
                    char hostname[255];
                    gethostname(hostname, 255);

                    // Get the command line arguments so we can find the current binary's name.
                    std::shared_ptr<const message::CommandLineArguments> args = store::DataStore<message::CommandLineArguments>::get();

                    std::vector<char> data(args->at(0).cbegin(), args->at(0).cend());
                    data.push_back('\0');
                    const auto* binary = basename(data.data());

                    // Set paths to the script files.
                    auto defaultConfig = "scripts/" + path;
                    auto robotConfig   = "scripts/" + std::string(hostname) + "/" + path;
                    auto binaryConfig  = "scripts/" + std::string(binary)   + "/" + path;

                    if (!utility::file::exists(defaultConfig))
                    {
                        throw std::runtime_error("Script file '" + defaultConfig + "' does not exist.");
                    }

                    auto defaultHandle = DSLProxy<::extension::FileWatch>::bind<DSL>(reactor, label, callback, defaultConfig, flags);
                    auto robotHandle   = utility::file::exists(robotConfig)
                                            ? DSLProxy<::extension::FileWatch>::bind<DSL>(reactor, label, callback, robotConfig, flags) 
                                            : threading::ReactionHandle();
                    auto binaryHandle  = utility::file::exists(binaryConfig)
                                            ? DSLProxy<::extension::FileWatch>::bind<DSL>(reactor, label, callback, binaryConfig, flags) 
                                            : threading::ReactionHandle();

                    // Set FileWatcher to monitor the requested files.
                    return std::make_tuple(defaultHandle, robotHandle, binaryHandle);
                }

                template <typename DSL>
                static inline std::shared_ptr<::extension::Script> get(threading::Reaction& t) {

                    // Get the file watch event
                    ::extension::FileWatch watch = DSLProxy<::extension::FileWatch>::get<DSL>(t);

                    // Check if the watch is valid
                    if(watch && utility::strutil::endsWith(watch.path, ".yaml")) {
                        // Return our yaml file
                        try {
                            // Get hostname so we can find the correct per-robot script directory.
                            char hostname[255];
                            gethostname(hostname, 255);

                            // Get the command line arguments so we can find the current binary's name.
                            std::shared_ptr<const message::CommandLineArguments> args = store::DataStore<message::CommandLineArguments>::get();

                            std::vector<char> data(args->at(0).cbegin(), args->at(0).cend());
                            data.push_back('\0');
                            const auto* binary = basename(data.data());

                            // Get relative path to script file.
                            auto components = utility::strutil::split(watch.path, '/');
                            std::string relativePath("");
                            bool flag = false;
                            for (const auto& component : components)
                            {
                                // Ignore the hostname/binary name if they are present.
                                if (flag && (component.compare(hostname) != 0) && (component.compare(binary) != 0))
                                {
                                    relativePath.append(component + "/");
                                }

                                // We want out paths relative to the script folder.
                                if (component.compare("scripts") == 0)
                                {
                                    flag = true;
                                }
                            }

                            // There will be a trailing / character.
                            relativePath.pop_back();

                            return std::make_shared<::extension::Script>(relativePath, hostname, binary);
                        } catch (const YAML::ParserException& e){
                            throw std::runtime_error(watch.path + " " + std::string(e.what()));
                        }
                    }
                    else {
                        // Return an empty configuration (which will show up invalid)
                        return std::shared_ptr<::extension::Script>(nullptr);
                    }
                }
            };
        }

        // Script is transient
        namespace trait {
            template <>
            struct is_transient<std::shared_ptr<::extension::Script>> : public std::true_type {};
        }
    }
}

namespace YAML {
    template<>
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
            rhs = { node["id"].as<std::string>()
                  , node["position"].as<float>(), node["gain"].as<float>()
                  , node["torque"] ? node["torque"].as<float>() : 100
                   };
            } catch(const YAML::Exception& e){
                NUClear::log<NUClear::ERROR>("Error parsing script -", "Line:", e.mark.line, "Column:", e.mark.column, "Pos:", e.mark.pos, "Message:", e.msg);
            }
            return true;
        }
    };

    template<>
    struct convert<::extension::Script::Frame> {
        static inline Node encode(const ::extension::Script::Frame& rhs) {
            Node node;

            node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(rhs.duration).count();
            node["targets"]  = rhs.targets;

            return node;
        }

        static inline bool decode(const Node& node, ::extension::Script::Frame& rhs) {

            int millis = node["duration"].as<int>();
            std::chrono::milliseconds duration(millis);

            std::vector<::extension::Script::Frame::Target> targets = node["targets"].as<std::vector<::extension::Script::Frame::Target>>();

            rhs = { duration, std::move(targets) };
            return true;
        }
    };

    template<>
    struct convert<::extension::Script> {
        static inline Node encode(const ::extension::Script& rhs) {
            Node node;

            node = rhs.frames;

            return node;
        }

        static inline bool decode(const Node& node, ::extension::Script& rhs) {
            std::vector<::extension::Script::Frame> frames = node.as<std::vector<::extension::Script::Frame>>();
            rhs = { std::move(frames) };
            return true;
        }
    };
}

#endif //EXTENSION_SCRIPT_H
