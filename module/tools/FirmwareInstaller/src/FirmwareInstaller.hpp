/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#ifndef MODULE_TOOLS_FIRMWAREINSTALLER_HPP
#define MODULE_TOOLS_FIRMWAREINSTALLER_HPP

#include <nuclear>
#include <string>

namespace module::tools {

    class FirmwareInstaller : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the FirmwareInstaller
        /// reactor.
        explicit FirmwareInstaller(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::string device;
        struct Firmware {
            std::vector<uint8_t> firmware;
            uint8_t checksum = 0;
        };

        std::map<std::pair<std::string, std::string>, Firmware> firmwares;

        enum MenuState { NO_MENU, DEVICE_MENU, BATTERY_MENU };
        enum Device { NO_DEVICE, CM730, CM740, DYNAMIXEL, OPENCR };
        enum Battery { NO_BATTERY, BATTERY3, BATTERY4 };
        MenuState menu_state;
        Device selected_device;
        Battery selected_battery;

        static void showDeviceMenu();
        static void showBatteryMenu();
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_FIRMWAREINSTALLER_HPP
