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
 * Copyright 2017 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_SCRIPT_H
#define UTILITY_SUPPORT_SCRIPT_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>

#include "extension/Script.h"

#include "utility/support/system.h"

namespace utility {
namespace support {

    /**
     * Save a script to a file
     *
     * @param file the file to write the new script to, if the file exists it will be overwritten
     * @param script the script to save
     *
     * @author Alex Biddulph
     */
    void saveScript(const std::string& file, const message::extension::Script& script) {
        std::ofstream yaml(fmt::format("scripts/{}/{}", getHostname(), file), std::ios::trunc | std::ios::out);
        yaml << YAML::convert<std::vector<message::extension::Script::Frame>>::encode(script.frames);
        yaml.close();
    }

    /**
     * Merge two scripts together into a new script
     *
     * The scripts will be merged in the order provided
     *
     * @param s1 the first script to merge
     * @param s2 the second script to merge
     *
     * @author Alex Biddulph
     */
    message::extension::Script operator+(const message::extension::Script& s1, const message::extension::Script& s2) {
        message::extension::Script s;
        s.frames.insert(s.frames.end(), s1.frames.begin(), s1.frames.end());
        s.frames.insert(s.frames.end(), s2.frames.begin(), s2.frames.end());
        return s;
    }

    /**
     * Merge two scripts together into a new script
     *
     * The scripts will be merged in the order provided
     *
     * @param scripts a list of scripts to be merged
     *
     * @author Alex Biddulph
     */
    message::extension::Script operator+(const std::vector<message::extension::Script>& scripts) {
        message::extension::Script s;
        for (const auto& script : scripts) {
            s.frames.insert(s.frames.end(), script.frames.begin(), script.frames.end());
        }
        return s;
    }

}  // namespace support
}  // namespace utility

#endif  // UTILITY_SUPPORT_SCRIPT_H
