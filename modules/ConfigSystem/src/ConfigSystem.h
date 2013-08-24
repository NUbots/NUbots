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
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_CONFIGSYSTEM_H_
#define MODULES_CONFIGSYSTEM_H_

#include <NUClear.h>
#include "messages/Configuration.h"

namespace modules {

	class ConfigSystem : public NUClear::Reactor {

	private:
		std::map<std::string, std::map<std::type_index,
		std::function<void (NUClear::Reactor*, messages::ConfigurationNode*)>>> configurations;

		std::map<int, std::string> wdMap;

		int watcherFd;
	public:
		explicit ConfigSystem(NUClear::PowerPlant* plant);
	} ;
}
#endif

