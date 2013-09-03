/*
 * This file is part of DarwinCameraReader.
 *
 * DarwinCameraReader is free software: you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * DarwinCameraReader is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinCameraReader.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Michael Burton <michael.burton@uon.edu.au>, Jake Woods <jake.f.woods@gmail.com>,
 * Trent Houliston <trent@houliston.me>
 */

#include "DarwinCameraReader.h"

#include "DarwinCamera.h"
#include "messages/Image.h"
#include "messages/Configuration.h"

namespace modules {

    // We assume that the device will always be video0, if not then change this
    DarwinCameraReader::DarwinCameraReader(NUClear::PowerPlant* plant) : Reactor(plant), camera("/dev/video0") {
        
        // This trigger gets us as close as we can to the frame rate as possible (as high resolution as we can)
        on<Trigger<Every<NUClear::clock::period::den / DarwinCamera::FRAMERATE, NUClear::clock::duration>>, Options<Single>>([this](const time_t& time) {
            
            // Get an image and emit it
            messages::Image* image = camera.getImage();
            emit(image);
        });

        // When we shutdown, we must tell our camera class to close (stop streaming)
        on<Trigger<Shutdown>>([this](const Shutdown& shutdown) {
            camera.closeCamera();
        });

        on<Trigger<messages::Configuration<DarwinCameraReader>>>([this](const messages::Configuration<DarwinCameraReader>& settings) {
            try {
                // Recreate the camera device at the required resolution
                camera.resetCamera(settings["imageWidth"], settings["imageHeight"]);
            
                // Set all other camera settings
                for(auto& setting : camera.getSettings()) {
                    int value = settings[setting.first];
                    setting.second.set(value);
                }
                std::cout << "Camera configuration applied." << std::endl;
            } catch(const std::exception& e) {
                std::cout << "Exception while setting camera configuration: " << e.what() << std::endl;
            }
        });
    }
}
