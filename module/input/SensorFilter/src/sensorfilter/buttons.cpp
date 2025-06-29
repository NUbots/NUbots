/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "message/input/Buttons.hpp"

#include "SensorFilter.hpp"

namespace module::input {

    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::platform::RawSensors;

    void SensorFilter::detect_button_press(const std::list<std::shared_ptr<const RawSensors>>& sensors) {
        // Keep track of the number of presses in the last N frames for debouncing
        int left_count   = 0;
        int middle_count = 0;

        // Count the number of downs in all messages we have
        //     we used to discount sensors messages with subcontroller errors here, but that
        //     ignored too many and ultimately led to unresponsive buttons. subcontroller errors
        //     are unlikely to cause phantom button presses, and if they do, the debounce routine
        //     would require that we have N_thresh > N errors causing the _same_ phantom button press
        for (const auto& s : sensors) {
            left_count += s->buttons.left ? 1 : 0;
            middle_count += s->buttons.middle ? 1 : 0;
        }

        // Compare to the debounce threshold to determine if we have a down event
        bool new_left_down   = left_count > cfg.button_debounce_threshold;
        bool new_middle_down = middle_count > cfg.button_debounce_threshold;
        // Check for a state change, i.e. a press or release event
        bool left_state_change = left_down != new_left_down;
        bool mid_state_change  = middle_down != new_middle_down;
        // And set the state variable for next time.
        left_down   = new_left_down;
        middle_down = new_middle_down;

        // If we have a state change, emit the appropriate event
        if (left_state_change) {
            if (left_down) {
                log<INFO>("Left Button Down");
                emit<Scope::INLINE>(std::make_unique<ButtonLeftDown>());
            }
            else {
                log<INFO>("Left Button Up");
                emit<Scope::INLINE>(std::make_unique<ButtonLeftUp>());
            }
        }
        if (mid_state_change) {
            if (middle_down) {
                log<INFO>("Middle Button Down");
                emit<Scope::INLINE>(std::make_unique<ButtonMiddleDown>());
            }
            else {
                log<INFO>("Middle Button Up");
                emit<Scope::INLINE>(std::make_unique<ButtonMiddleUp>());
            }
        }
    }
}  // namespace module::input
