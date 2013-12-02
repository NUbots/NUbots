/*
 * This file is part of MechWarrior.
 *
 * MechWarrior is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MechWarrior is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MechWarrior.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "MechWarrior.h"
#include "messages/DarwinSensors.h"

namespace modules {
    namespace behaviours {

        MechWarrior::MechWarrior(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), fired(0) {

            on<Trigger<Every<250, std::chrono::milliseconds>>>([this](const time_t&) {
                if(++fired < 7) {
                    emit(std::make_unique<messages::LMissile>());
                }
                else {
                    powerPlant->shutdown();
                }
            });
        }
    }
}