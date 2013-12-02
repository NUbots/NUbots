/*
 * This file is part of DanceDarwin.
 *
 * DanceDarwin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DanceDarwin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DanceDarwin.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "DanceDarwin.h"

#include "messages/ServoWaypoint.h"
#include "messages/Configuration.h"
#include "messages/Beat.h"

namespace modules {
    namespace Behaviours {
        struct DanceScripts {
            // For scripts we want updates on the whole scripts directory
            static constexpr const char* CONFIGURATION_PATH = "scripts/dance/";
        };

        DanceDarwin::DanceDarwin(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<messages::Configuration<DanceScripts>>>([this](const messages::Configuration<DanceScripts>& script) {
                // Add this script to our list of scripts
                scripts.insert(std::make_pair(script.name, script.config));
            });


            on<Trigger<messages::AllServoWaypointsComplete>, With<messages::Beat>>([this](const messages::AllServoWaypointsComplete&, const messages::Beat& beat) {
                std::cout << "ServoWaypointsComplete" << std::endl;
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
                auto start1 = beat.time + beat.period + (beat.period * (std::chrono::duration_cast<std::chrono::nanoseconds>(NUClear::clock::now() - beat.time).count() / std::chrono::duration_cast<std::chrono::nanoseconds>(beat.period).count()));
                auto start2 = beat.time + (beat.period * (std::chrono::duration_cast<std::chrono::nanoseconds>(NUClear::clock::now() - beat.time).count() / std::chrono::duration_cast<std::chrono::nanoseconds>(beat.period).count()));

                auto start = abs(std::chrono::duration_cast<std::chrono::nanoseconds>(beat.time - start1).count()) > abs(std::chrono::duration_cast<std::chrono::nanoseconds>(beat.time - start2).count()) ? start2 : start1;
                start = start1;

                // Work out the smallest multiple of 2 multiplied by period that is greater then duration
                // By using a power of two, we assume that the songs time signature is even (3/4 songs will look strange)
                auto durationInNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
                auto beatInNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(beat.period);
                auto targetLengthInNanoseconds = exp2(ceil(log2(
                    double(durationInNanoseconds.count()) /
                    double(beatInNanoseconds.count())
                ))) * beatInNanoseconds;

                // Work out how much we need to scale our script by to make it fit into our beat
                double scale = double(targetLengthInNanoseconds.count()) / double(durationInNanoseconds.count());
                std::cout << "Scaling script by: " << scale << std::endl;

                // Scale our script
                messages::Script script;
                for(const auto& frame : item->second.frames) {
                    script.frames.push_back(frame);
                    script.frames.back().duration *= scale;
                }

                // Emit our scaled script to start at our start time (normally in the past)
                emit(std::make_unique<messages::ExecuteScript>(script, start));
            });

            // Awful hack do not use.
            on<Trigger<messages::Beat>>([this](const messages::Beat& beat) {
                if(!startedDancing) {
                    emit(std::make_unique<messages::ExecuteScriptByName>("Stand.json"));
                    startedDancing = true;
                }
            });
        }
    }
}
