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

#include "NUsight.h"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include "utility/file/fileutil.h"
#include "utility/strutil/strutil.h"

/**
 * @author Monica Olejniczak
 */
namespace module {
namespace support {
    using utility::file::listFiles;
    using utility::strutil::split;

    /**
     * @brief Saves the configuration file using the root YAML node.
     * @details Writes to the YAML file specified at the path by using a YAML emitter. This emitter takes the output
     * stream of the
     * root YAML node and saves it to the file.
     *
     * @param path The path to the configuration file.
     * @param root The root YAML node.
     */
    void NUsight::saveConfigurationFile(std::string path, const std::string& root) {
        std::string tempName(fmt::format("{}.tmp", path));
        utility::file::writeToFile(tempName, root);
        rename(tempName.c_str(), path.c_str());

        // YAML::Emitter emitter;          // create a YAML emitter
        // emitter << root;                // send the root node to the emitter's output stream
        // std::ofstream fout(path);       // create an output stream to the specified path
        // fout << emitter.c_str();        // write to the file
    }
}  // namespace support
}  // namespace module
