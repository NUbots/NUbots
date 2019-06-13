/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MOTION_SCRIPT_H
#define UTILITY_MOTION_SCRIPT_H

#include "message/motion/script/Script.h"

namespace utility {
namespace motion {
    namespace script {
        using message::motion::script::Script;

        inline std::string getHostname() {
            // Get hostname so we can find the correct per-robot script directory.
            char host[255];
            gethostname(host, 255);
            return host;
        }

        inline std::string getPlatform(const std::string& hostname) {
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

        inline Script createScript(const std::string& filename) {
            Script script;

            script.filename = filename;
            script.hostname = getHostname();
            script.platform = getPlatform(script.hostname);

            return script;
        }

        inline Script loadScript(const std::string& filename) {
            Script script;

            script.filename = filename;
            script.hostname = getHostname();
            script.platform = getPlatform(script.hostname);

            YAML::Node yaml;

            // Per robot scripts:    Scripts that are specific to a certain robot (e.g. darwin1).
            //                       These are to account for minor hardware variations in a robot and, as such, take
            //                       precedence over per platform scripts.
            if (utility::file::exists("scripts/" + script.hostname + "/" + script.filename)) {
                NUClear::log<NUClear::INFO>("Parsing robot specific script:", script.filename);
                yaml = YAML::LoadFile("scripts/" + script.hostname + "/" + script.filename);
            }

            // Per platform scripts: Scripts that are specific to a certain platform (e.g. darwin, igus, etc).
            //                       These are the default scripts, it is an error for this version of the script to not
            //                       exist.
            else if (utility::file::exists("scripts/" + script.platform + "/" + script.filename)) {
                NUClear::log<NUClear::INFO>("Parsing default platform script:", script.filename);
                yaml = YAML::LoadFile("scripts/" + script.platform + "/" + script.filename);
            }

            script.servos = yaml.as<std::vector<Script::Servo>>();

            return script;
        }

        inline void saveScript(const Script& script) {
            std::ofstream file("scripts/" + script.hostname + "/" + script.filename, std::ios::trunc | std::ios::out);
            file << YAML::convert<std::vector<Script::Servo>>::encode(script.servos);
            file.close();
        }
    }  // namespace script
}  // namespace motion
}  // namespace utility

#endif  // UTILITY_MOTION_SCRIPT_H
