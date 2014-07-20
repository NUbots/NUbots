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

#ifndef MODULES_PLATFORM_DARWIN_SENSORFILTER_H
#define MODULES_PLATFORM_DARWIN_SENSORFILTER_H

#include <nuclear>

#include "utility/math/kalman/UKF.h"
#include "utility/math/kalman/IMUModel.h"
#include "utility/math/kalman/LinearVec3Model.h"
#include "utility/motion/RobotModels.h"
#include "messages/input/Sensors.h"

namespace modules {
    namespace platform {
        namespace darwin {

            /**
             * TODO document
             *
             * @author Jake Fountain
             * @author Trent Houliston
             */
            class SensorFilter : public NUClear::Reactor {
            public:
                explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

                utility::math::kalman::UKF<utility::math::kalman::IMUModel> orientationFilter;
                utility::math::kalman::UKF<utility::math::kalman::LinearVec3Model> velocityFilter;

                double DEFAULT_NOISE_GAIN;
                double HIGH_NOISE_THRESHOLD;
                double HIGH_NOISE_GAIN;
                double LOW_NOISE_THRESHOLD;
                int DEBOUNCE_THRESHOLD;

                double SUPPORT_FOOT_FSR_THRESHOLD;
                int REQUIRED_NUMBER_OF_FSRS;

                arma::mat MEASUREMENT_NOISE_ACCELEROMETER;
                arma::mat MEASUREMENT_NOISE_GYROSCOPE;

                double odometry_covariance_factor = 0.05;

                arma::vec2 integratedOdometry;

                static constexpr const char* CONFIGURATION_PATH = "DarwinSensorFilter.yaml";
            private:
                arma::mat44 calculateOdometryMatrix(
                    const messages::input::Sensors& sensors,
                    const messages::input::Sensors& previousSensors,
                    utility::motion::kinematics::Side side);

                // used to debounce button presses
                bool leftDown = false;
                bool middleDown = false;
            };
        }
    }
}
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_H

