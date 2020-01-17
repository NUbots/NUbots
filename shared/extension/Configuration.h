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

#ifndef EXTENSION_CONFIGURATION_H
#define EXTENSION_CONFIGURATION_H

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <nuclear>

#include "FileWatch.h"
#include "utility/file/fileutil.h"
#include "utility/strutil/strutil.h"

namespace extension {

/**
 * TODO document
 *
 * @author Trent Houliston
 */
struct Configuration {
    // Rules:
    // 1) Default config file should define a value for every node.
    // 2) Per-robot config overrides default config values. This file need only override the values that nee to be
    // overriden.
    // 3) Per-binary config overrides per-robot and default config values. This file need only override the values that
    // nee to be overriden.
    //
    // Per-robot and per-binary files need not exist.
    // Per-robot and per-binary files can add new nodes to the file, but this is probably unwise.
    //
    // We have to merge the YAML trees to account for situations where a sub-node is not defined in a higher priotity
    // tree.

    std::string fileName, hostname, binary;
    YAML::Node config;

    Configuration() : fileName(), hostname(), binary(), config(){};
    Configuration(const std::string& fileName,
                  const std::string& hostname,
                  const std::string& binary,
                  const YAML::Node& config)
        : fileName(fileName), hostname(hostname), binary(binary), config(config) {}

    Configuration(const std::string& fileName, const std::string& hostname, const std::string& binary)
        : fileName(fileName), hostname(hostname), binary(binary), config() {
        bool loaded = false;

        // Load the default config file.
        if (utility::file::exists("config/" + fileName)) {
            config = YAML::LoadFile("config/" + fileName);
            loaded = true;
        }

        // If the same file exists in this robots per-robot config directory then load and merge.
        if (utility::file::exists("config/" + hostname + "/" + fileName)) {
            if (loaded) {
                config = mergeYAML(config, YAML::LoadFile("config/" + hostname + "/" + fileName));
            }

            else {
                config = YAML::LoadFile("config/" + hostname + "/" + fileName);
                loaded = true;
            }
        }

        // If the same file exists in this binary's per-binary config directory then load and merge.
        if (utility::file::exists("config/" + binary + "/" + fileName)) {
            if (loaded) {
                config = mergeYAML(config, YAML::LoadFile("config/" + binary + "/" + fileName));
            }

            else {
                config = YAML::LoadFile("config/" + binary + "/" + fileName);
            }
        }
    }

    YAML::Node mergeYAML(const YAML::Node& base, const YAML::Node& override) {
        YAML::Node ret(base);

        for (auto it = override.begin(); it != override.end(); it++) {
            const std::string& key = it->first.as<std::string>();

            // If the key doesn't exist in the base set then add it and move on.
            if (!base[key]) {
                ret[key] = it->second;
            }

            // If the key does exist then we must go deeper.
            else {
                // The type dictates whether we need a recursive call or not.
                switch (it->second.Type()) {
                    // Just a raw value (int, double, etc)
                    case YAML::NodeType::Scalar: {
                        ret[key] = it->second;
                        break;
                    }

                    // Essentially a vector.
                    case YAML::NodeType::Sequence: {
                        ret[key] = it->second;
                        break;
                    }

                    // Recurse.
                    case YAML::NodeType::Map: {
                        ret[key] = mergeYAML(base[key], override[key]);
                        break;
                    }

                    // Its nothing, so overwrite (unset) the value.
                    // Is this really the intended behaviour?
                    case YAML::NodeType::Null:
                    case YAML::NodeType::Undefined:
                    default: {
                        NUClear::log<NUClear::WARN>(
                            "Unsetting key", "'" + key + "'", "in YAML file. Is this what you intended?");
                        ret[key] = it->second;
                        break;
                    }
                }
            }
        }

        return ret;
    }

    Configuration operator[](const std::string& key) {
        return Configuration(fileName, hostname, binary, config[key]);
    }

    const Configuration operator[](const std::string& key) const {
        return Configuration(fileName, hostname, binary, config[key]);
    }

    Configuration operator[](const char* key) {
        return Configuration(fileName, hostname, binary, config[key]);
    }

    const Configuration operator[](const char* key) const {
        return Configuration(fileName, hostname, binary, config[key]);
    }

    Configuration operator[](size_t index) {
        return Configuration(fileName, hostname, binary, config[index]);
    }

    const Configuration operator[](size_t index) const {
        return Configuration(fileName, hostname, binary, config[index]);
    }

    Configuration operator[](int index) {
        return Configuration(fileName, hostname, binary, config[index]);
    }

    const Configuration operator[](int index) const {
        return Configuration(fileName, hostname, binary, config[index]);
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
        struct DSLProxy<::extension::Configuration> {
            template <typename DSL>
            static inline void bind(const std::shared_ptr<threading::Reaction>& reaction, const std::string& path) {
                auto flags = ::extension::FileWatch::RENAMED | ::extension::FileWatch::CHANGED;

                // Get hostname so we can find the correct per-robot config directory.
                char hostname[255];
                gethostname(hostname, 255);

                // Get the command line arguments so we can find the current binary's name.
                std::shared_ptr<const message::CommandLineArguments> args =
                    store::DataStore<message::CommandLineArguments>::get();

                std::vector<char> data(args->at(0).cbegin(), args->at(0).cend());
                data.push_back('\0');
                const auto* binary = basename(data.data());

                // Set paths to the config files.
                auto defaultConfig = "config/" + path;
                auto robotConfig   = "config/" + std::string(hostname) + "/" + path;
                auto binaryConfig  = "config/" + std::string(binary) + "/" + path;

                if (!utility::file::exists(defaultConfig)) {
                    NUClear::log<NUClear::WARN>("Configuration file '" + defaultConfig
                                                + "' does not exist. Creating it.");

                    // Check for a directory.
                    // If the path ends in a /, or if the end of the string is not ".yaml" it is a directory.
                    if ((defaultConfig.back() == '/') || (!utility::strutil::endsWith(defaultConfig, ".yaml"))) {
                        utility::file::makeDir(defaultConfig);
                    }

                    else {
                        std::ofstream ofs(defaultConfig);
                        if (!ofs.is_open()) {
                            throw std::runtime_error("Failed creating file '" + path + "'.");
                        }
                        ofs.close();
                    }
                }

                // Bind our default path
                DSLProxy<::extension::FileWatch>::bind<DSL>(reaction, defaultConfig, flags);

                // Bind our robot specific path if it exists
                if (utility::file::exists(robotConfig)) {
                    DSLProxy<::extension::FileWatch>::bind<DSL>(reaction, robotConfig, flags);
                }

                // Bind our binary specific path if it exists
                if (utility::file::exists(binaryConfig)) {
                    DSLProxy<::extension::FileWatch>::bind<DSL>(reaction, binaryConfig, flags);
                }
            }

            template <typename DSL>
            static inline std::shared_ptr<::extension::Configuration> get(threading::Reaction& t) {

                // Get the file watch event
                ::extension::FileWatch watch = DSLProxy<::extension::FileWatch>::get<DSL>(t);

                // Check if the watch is valid
                if (watch && utility::strutil::endsWith(watch.path, ".yaml")) {
                    // Return our yaml file
                    try {
                        // Get hostname so we can find the correct per-robot config directory.
                        char hostname[255];
                        gethostname(hostname, 255);

                        // Get the command line arguments so we can find the current binary's name.
                        std::shared_ptr<const message::CommandLineArguments> args =
                            store::DataStore<message::CommandLineArguments>::get();

                        std::vector<char> data(args->at(0).cbegin(), args->at(0).cend());
                        data.push_back('\0');
                        const auto* binary = basename(data.data());

                        // Get relative path to config file.
                        auto components = utility::strutil::split(watch.path, '/');
                        std::string relativePath("");
                        bool flag = false;
                        for (const auto& component : components) {
                            // Ignore the hostname/binary name if they are present.
                            if (flag && (component.compare(hostname) != 0) && (component.compare(binary) != 0)) {
                                relativePath.append(component + "/");
                            }

                            // We want out paths relative to the config folder.
                            if (component.compare("config") == 0) {
                                flag = true;
                            }
                        }

                        // There will be a trailing / character.
                        relativePath.pop_back();

                        return std::make_shared<::extension::Configuration>(relativePath, hostname, binary);
                    }
                    catch (const YAML::ParserException& e) {
                        throw std::runtime_error(watch.path + " " + std::string(e.what()));
                    }
                }
                else {
                    // Return an empty configuration (which will show up invalid)
                    return std::shared_ptr<::extension::Configuration>(nullptr);
                }
            }
        };
    }  // namespace operation

    // Configuration is transient
    namespace trait {
        template <>
        struct is_transient<std::shared_ptr<::extension::Configuration>> : public std::true_type {};
    }  // namespace trait
}  // namespace dsl
}  // namespace NUClear

#endif  // EXTENSION_CONFIGURATION_H
