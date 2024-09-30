/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "HardwareIO.hpp"

#include "utility/math/angle.hpp"

namespace module::platform::OpenCR {

    using message::platform::RawSensors;

    RawSensors HardwareIO::construct_sensors() {
        RawSensors sensors;

        // Timestamp when this message was created (data itsself could be old)
        sensors.timestamp = NUClear::clock::now();

        /* OpenCR data */
        sensors.subcontroller_error = opencr_state.packet_error;
        sensors.led_panel           = opencr_state.led_panel;
        sensors.head_led            = opencr_state.head_led;
        sensors.eye_led             = opencr_state.eye_led;
        sensors.buttons             = opencr_state.buttons;
        sensors.accelerometer       = opencr_state.acc;
        sensors.gyroscope           = opencr_state.gyro;

        /* Battery data */
        sensors.battery = battery_state.current_voltage;

        /* Servos data */
        sensors.servos = servos;

        return sensors;
    }

}  // namespace module::platform::OpenCR
