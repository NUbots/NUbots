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
#include "messages/support/Configuration.h"
#include "utility/NUbugger/NUgraph.h"
#include "messages/localisation/FieldObject.h"

using utility::NUbugger::graph;

namespace modules {
    Localisation::Localisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        on<Trigger<messages::support::Configuration<Localisation>>>([this](const messages::support::Configuration<Localisation>& settings) {
            std::cout << __func__ << ": Config" << std::endl;
            
            std::string testConfig = settings.config["testConfig"];
            std::cout << testConfig << std::endl;
        });

    	on<Trigger<Every<500, std::chrono::milliseconds>>>([this](const time_t&) {
            // emit(std::make_unique<messages::LMissile>());
            std::cout << __PRETTY_FUNCTION__ << ": rand():" << rand() << std::endl;

            auto field_object = std::make_unique<messages::localisation::FieldObject>();
            field_object->name = "ball";
            field_object->wm_x = static_cast<float>(rand() % 400 - 200);
            field_object->wm_y = static_cast<float>(rand() % 600 - 300);
            field_object->sd_x = 100;
            field_object->sd_y = 25;
            field_object->sr_xx = 100;
            field_object->sr_xy = -1;
            field_object->sr_yy = 10;
            field_object->lost = false;

            emit(std::move(field_object));
        });

        // on<Trigger<messages::LMissile>>([this](const messages::LMissile&) {
        //     std::cout << __PRETTY_FUNCTION__ << ": Missile!" << std::endl;
        // });
    }
}
