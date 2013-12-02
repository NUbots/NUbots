/*
 * This file is part of Localisation.
 *
 * Localisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Localisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Localisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Localisation.h"
#include "messages/DarwinSensors.h"
#include "messages/Configuration.h"

namespace modules {
    Localisation::Localisation(NUClear::PowerPlant* plant) : Reactor(plant) {
        on<Trigger<messages::Configuration<Localisation>>>([this](const messages::Configuration<Localisation>& settings) {
            std::cout << __PRETTY_FUNCTION__ << ": Config" << std::endl;
            
            std::string testConfig = settings.config["testConfig"];
            std::cout << testConfig << std::endl;
        });

    	on<Trigger<Every<1000, std::chrono::milliseconds>>>([this](const time_t&) {
            std::cout << __PRETTY_FUNCTION__ << ": before missile" << std::endl;
            emit(std::make_unique<messages::LMissile>());
            std::cout << __PRETTY_FUNCTION__ << ": after missile" << std::endl;
        });

        on<Trigger<messages::LMissile>>([this](const messages::LMissile&) {
            std::cout << __PRETTY_FUNCTION__ << ": Missile!" << std::endl;
        });
    }
}

