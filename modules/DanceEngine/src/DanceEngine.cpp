/*
 * This file is part of DanceEngine.
 *
 * DanceEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DanceEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DanceEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "DanceEngine.h"

#include "messages/ServoWaypoint.h"
#include "messages/Configuration.h"
#include "messages/Beat.h"

namespace modules {
    struct DanceScripts {
        // For scripts we want updates on the whole scripts directory
        static constexpr const char* CONFIGURATION_PATH = "scripts/dance/";
    };

    DanceEngine::DanceEngine(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::Configuration<DanceScripts>>>([this](const messages::Configuration<DanceScripts>& script) {
            // Add this script to our list of scripts
            scripts.insert(std::make_pair(script.name, script.config));
        });

        on<Trigger<messages::AllServoWaypointsComplete>, With<messages::Beat>>([this](const messages::AllServoWaypointsComplete&, const messages::Beat& beat) {

            // Here we pick a random element. Note that the random selection is bias here however until the number
            // of scripts in the system is statistically significant compared to RAND_MAX, this should give decent results
            auto item = scripts.begin();
            std::advance(item, rand() % scripts.size());

            // Get the total duration of this script
            NUClear::clock::duration duration(0);
            for(const auto& frame : item->second.frames) {
                duration += frame.duration;
            }

            // Work out how many periods we need to add to beat.time to make it the one before now (could be 0)
            auto start = beat.time + (beat.period * ((NUClear::clock::now() - beat.time) / beat.period));

            // Work out the smallest multiple of 2 multiplied by period that is greater then duration
            // By using a power of two, we assume that the songs time signature is even (3/4 songs will look strange)
            auto length = exp2(ceil(log2(double(duration.count())/double(beat.period.count())))) * beat.period;

            // Work out how much we need to scale our script by to make it fit into our beat
            double scale = double(duration.count()) / double(length.count());

            // Scale our script
            messages::Script script;
            for(const auto& frame : item->second.frames) {
                script.frames.push_back(frame);
                script.frames.back().duration *= scale;
            }

            // Emit our scaled script to start at our start time (normally in the past)
            emit(std::make_unique<messages::ExecuteScript>(script, start));
        });

        on<Trigger<messages::Beat>>([this](const messages::Beat& beat) {
            // TODO think about how to strobe the LEDs
        });
    }
}
