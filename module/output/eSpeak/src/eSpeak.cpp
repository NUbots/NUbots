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

#include "eSpeak.h"

#include <espeak/speak_lib.h>

#include "message/output/Say.h"

namespace module {
namespace output {

    using message::output::Say;

    eSpeak::eSpeak(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Initialize espeak, and set it to play out the speakers, and not exit if it can't find it's directory
        espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 500, nullptr, 1 << 15);
        espeak_SetVoiceByName("default");
        espeak_SetParameter(espeakVOLUME, 100, 0);
        espeak_SetParameter(espeakCAPITALS, 6, 0);

        on<Trigger<Say>, Sync<eSpeak>>().then([](const Say& say) {
            // Wait to finish the current message (if any)
            // By waiting here this reaction can finish and return to the pool
            // if it does not have to wait for another say message
            espeak_Synchronize();

            // Say the new message
            espeak_Synth(say.message.c_str(),     // Text
                         say.message.size() + 1,  // Size (including null at end)
                         0,                       // Start position
                         POS_CHARACTER,           // Position Type (irrelevant since we start at the beginning)
                         0,                       // End position (0 means no end position)
                         espeakCHARS_AUTO,        // Flags (auto encoding)
                         nullptr,                 // User identifier for callback
                         nullptr                  // Callback
            );
        });

        on<Shutdown>().then(espeak_Terminate);
    }

}  // namespace output
}  // namespace module
