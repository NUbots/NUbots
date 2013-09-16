/*
 * This file is part of LinuxCameraStreamer.
 *
 * LinuxCameraStreamer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LinuxCameraStreamer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LinuxCameraStreamer.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "LinuxCameraStreamer.h"
#include "utility/idiom/pimpl_impl.h"

#include "V4L2Camera.h"
#include "messages/Image.h"
#include "messages/Configuration.h"

namespace modules {
    // Create our impl class as per the pimpl idiom.
    class LinuxCameraStreamer::impl {
        public:
            /// @brief Our internal camera class that interacts with the physical device
            V4L2Camera camera;
    };


    // We assume that the device will always be video0, if not then change this
    LinuxCameraStreamer::LinuxCameraStreamer(NUClear::PowerPlant* plant) : Reactor(plant) {

        // This trigger gets us as close as we can to the frame rate as possible (as high resolution as we can)
        on<Trigger<Every<NUClear::clock::period::den / V4L2Camera::FRAMERATE, NUClear::clock::duration>>, Options<Single>>([this](const time_t& time) {

            // Get an image and emit it
            emit(std::unique_ptr<messages::Image>(m->camera.getImage()));
        });

        // When we shutdown, we must tell our camera class to close (stop streaming)
        on<Trigger<Shutdown>>([this](const Shutdown& shutdown) {
            m->camera.closeCamera();
        });

        on<Trigger<messages::Configuration<LinuxCameraStreamer>>>([this](const messages::Configuration<LinuxCameraStreamer>& settings) {
            auto& camera = m->camera;

            try {
                // Recreate the camera device at the required resolution
                int width = settings.config["imageWidth"];
                int height = settings.config["imageHeight"];
                std::string deviceID = settings.config["deviceID"];
                if (camera.getWidth() != static_cast<size_t>(width)
                    || camera.getHeight() != static_cast<size_t>(height)
                    || camera.getDeviceID() != deviceID) {
                    camera.resetCamera(deviceID, width, height);
                }

                // Set all other camera settings
                for(auto& setting : camera.getSettings()) {
                    int value = settings.config[setting.first];
                    setting.second.set(value);
                }
            } catch(const std::exception& e) {
                std::cout << "Exception while setting camera configuration: " << e.what() << std::endl;
            }
        });
    }
}
