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

#include "message/input/Sensors.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/filter/UKF.h"
#include "IMUModel.h"
#include "utility/motion/RobotModels.h"

namespace module {
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

                utility::math::filter::UKF<IMUModel> orientationFilter;

                double DEFAULT_NOISE_GAIN;
                double HIGH_NOISE_THRESHOLD;
                double HIGH_NOISE_GAIN;
                double LOW_NOISE_THRESHOLD;
                int DEBOUNCE_THRESHOLD;

                double SUPPORT_FOOT_FSR_THRESHOLD;
                int REQUIRED_NUMBER_OF_FSRS;

                arma::mat33 MEASUREMENT_NOISE_ACCELEROMETER;
                arma::mat33 MEASUREMENT_NOISE_GYROSCOPE;
                arma::mat33 MEASUREMENT_NOISE_FOOT_UP;
                double FOOT_UP_SAFE_ZONE;

                double odometry_covariance_factor = 0.05;

                arma::vec2 integratedOdometry;

            private:
                utility::math::matrix::Transform3D calculateOdometryMatrix(
                    const message::input::Sensors& sensors,
                    const message::input::Sensors& previousSensors,
                    utility::motion::kinematics::Side side);

                // used to debounce button presses
                bool leftDown = false;
                bool middleDown = false;
            };
        }
    }
}
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_H

