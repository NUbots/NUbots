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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_DARWIN_HARDWARESIMULATOR_H
#define MODULES_PLATFORM_DARWIN_HARDWARESIMULATOR_H

#include <nuclear>
#include <armadillo>

#include "messages/platform/darwin/DarwinSensors.h"

namespace modules {
namespace platform {
namespace darwin {

    /**
     * This NUClear Reactor is responsible for reading in the data for the Darwin Platform and emitting it to the rest
     * of the system
     *
     * @author Jake Fountain
     */
    class HardwareSimulator : public NUClear::Reactor {
    private:
        messages::platform::darwin::DarwinSensors sensors;
        std::queue<messages::platform::darwin::DarwinSensors::Gyroscope> gyroQueue;
        float imu_drift_rate;
        static constexpr size_t UPDATE_FREQUENCY = 90;
        void addNoise(std::unique_ptr<messages::platform::darwin::DarwinSensors>& sensors);
        struct NoiseConfig{
            struct Vec3Noise{
                float x = 0.001;
                float y = 0.001;
                float z = 0.001;
            };
            Vec3Noise accelerometer;
            Vec3Noise gyroscope;
        } noise;
        double bodyTilt;
        arma::vec3 integrated_gyroscope;
        void setRightFootDown();
        void setLeftFootDown();
        void setBothFeetDown();
    public:
        /// @brief called by a Powerplant to construct this reactor
        explicit HardwareSimulator(std::unique_ptr<NUClear::Environment> environment);
        static constexpr const char* CONFIGURATION_PATH = "DarwinHardwareSimulator.yaml";
    };
}
}
}
#endif

