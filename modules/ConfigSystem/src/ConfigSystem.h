/*
 * This file is part of ConfigSystem.
 *
 * ConfigSystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ConfigSystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ConfigSystem.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_CONFIGSYSTEM_H_
#define MODULES_CONFIGSYSTEM_H_

#include <NUClear.h>
#include <vector>
#include <set>
#include <map>

#include "messages/Configuration.h"

namespace modules {

    /**
     * TODO document
     *
     * @author Trent Houliston
     * @author Michael Burton
     */
    class ConfigSystem : public NUClear::Reactor {

    private:
        using HandlerFunction = std::function<void (NUClear::Reactor*, const std::string&, const messages::ConfigurationNode&)>;

        static constexpr const char* BASE_CONFIGURATION_PATH = "config/";

        std::set<std::type_index> loaded;
        std::map<std::string, std::vector<HandlerFunction>> handler;
        std::map<int, std::string> watchPath;
        std::map<std::string, NUClear::clock::time_point> timestamp;

        volatile bool running;
        void run();
        void kill();
        void watchDir(const std::string& path, HandlerFunction emitter, HandlerFunction emitNow);
        messages::ConfigurationNode buildConfigurationNode(const std::string& filePath);

        int watcherFd;
        int killFd;
    public:
        explicit ConfigSystem(NUClear::PowerPlant* plant);
    };
}
#endif

