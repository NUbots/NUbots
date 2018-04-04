/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */
#include "PostureRecogniser.h"

namespace module {
namespace input {

    using extension::Configuration;

    using message::input::BendingDetected;
    using message::input::FallingDetected;
    using message::input::KickingDetected;
    using message::input::Sensors;
    using message::input::SittingDetected;
    using message::input::StandingDetected;
    using message::input::WalkingDetected;

    using utility::input::ServoLoadModel;
    using utility::math::filter::UKF;
    using utility::nusight::graph;
    using utility::time::TimeDifferenceSeconds;

    /*=======================================================================================================*/
    //      NAME: Posture Recogniser
    /*=======================================================================================================*/
    PostureRecogniser::PostureRecogniser(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , loadFilters()
        , lastTimeUpdateTime()
        , DEBUG(false)
        , DEBUG_ITER(0)
        , emitAccelerometer(false)
        , emitGyroscope(false)
        , emitFallingScaleFactor(false) {

        // Configure posture recognition...
        on<Configuration>("PostureRecogniser.yaml").then([this](const Configuration& config) {
            configure(config.config);
        });

        // If there is some impulse relating robots posture and orientation, then capture values for processing...
        on<Last<2, Trigger<Sensors>>>().then([this](const std::vector<std::shared_ptr<const Sensors>>& sensors) {
            // Only continue if there is at least 2 sets of sensor data...
            if (DEBUG) {
                log<NUClear::TRACE>("Posture Recogniser - Enter Trigger(0)");
            }
            if (sensors.size() < 2) {
                return;
            }
            if (DEBUG) {
                log<NUClear::TRACE>("Posture Recogniser- Enter Trigger(1)");
            }

            // The acceleration amplitude for stationary postures is typically smaller than 0.40g...
            // The rotational rate amplitude for stationary postures typically is smaller than 1.0472 rads/s...
            // Evaluate thresholds for dynamic postures and abrupt noise (such as falling, jumping, etc.):
            // Acceleration amplitude = 3.0g...
            // Rotational rate = 3.49066 rads/sec...
            // Bending vs. Standing is considered at an offset of 0.610865 rads...

            // Gyroscope (in radians/second)
            // Capture axis differences in gyroscope data...
            arma::vec3 gyroDiff   = convert<double, 3>(sensors[0]->gyroscope - sensors[1]->gyroscope);
            arma::vec2 xzGyroDiff = {gyroDiff(0), gyroDiff(2)};
            arma::vec2 yzGyroDiff = {gyroDiff(1), gyroDiff(2)};
            arma::vec2 xyGyroDiff = {gyroDiff(0), gyroDiff(1)};

            // DEBUG: Accelerometer data...
            if (emitAccelerometer) {
                emit(graph("xyGyroDiff", xyGyroDiff));
                emit(graph("xzGyroDiff", xzGyroDiff));
                emit(graph("yzGyroDiff", yzGyroDiff));
            }

            // Accelerometer (in m/s^2)
            // Capture axis differences in accelerometer data...
            arma::vec3 accelDiff   = convert<double, 3>(sensors[0]->accelerometer - sensors[1]->accelerometer);
            arma::vec2 xzAccelDiff = {accelDiff(0), accelDiff(2)};
            arma::vec2 yzAccelDiff = {accelDiff(1), accelDiff(2)};
            arma::vec2 xyAccelDiff = {accelDiff(0), accelDiff(1)};

            // DEBUG: Gyroscope data...
            if (emitGyroscope) {
                emit(graph("xyAccelDiff", xyAccelDiff));
                emit(graph("xzAccelDiff", xzAccelDiff));
                emit(graph("yzAccelDiff", yzAccelDiff));
            }

            // Robot Orientation (cosine of torso anglular position)
            // Consider a normalisation of the sensor values to a unitless severity scale...
            double fallingScaleY = 0;
            if ((sensors[1]->world(2, 2)) < 0.915)  // TODO : Make these parameters configurable...
            {
                // Simplified 1 + (((sensors[1]->world(2,2) - 0.5)(0-1))/(0.915-0.5));
                fallingScaleY = (2.40964 * (0.5 - sensors[1]->world(2, 2))) + 1;
            }
            // Enclosure scaling values to [0,1]...
            fallingScaleY = (fallingScaleY > 1.0) ? 1.0 : ((fallingScaleY < -1.0) ? -1.0 : fallingScaleY);

            // Simplified 1 + ((((sensors[1]->world(1,2)) - 0.6)(1--1))/(0.6--0.6));
            double fallingScaleX = (1.66667 * ((sensors[1]->world(1, 2)) - 0.6)) + 1;
            // Enclosure scaling values to [0,1]...
            fallingScaleX = (fallingScaleX > 1.0) ? 1.0 : ((fallingScaleX < -1.0) ? -1.0 : fallingScaleX);

            // DEBUG: FallingDetected data...
            if (emitFallingScaleFactor) {
                emit(graph("Falling Detected Scaling Factor", fallingScaleX, fallingScaleY));
            }

            // Notify if sensor data suggests robot is walking...  TODO: likely requies greater collection of sensor
            // data
            // WalkingDetected walking;
            //    ...
            // emit(std::make_unique<WalkingDetected>(walking));

            // Notify if sensor data suggests robot is bending over...  TODO: likely requies greater collection of
            // sensor data
            // BendingDetected bending;
            //    ...
            // emit(std::make_unique<BendingDetected>(bending));

            // Notify if sensor data suggests robot is kicking...  TODO: likely requies greater collection of sensor
            // data
            // KickingDetected kicking;
            //    ...
            // emit(std::make_unique<KickingDetected>(kicking));

            // Notify if sensor data suggests robot is sitting down...  TODO: likely requies greater collection of
            // sensor data
            // SittingDetected sitting;
            //    ...
            // emit(std::make_unique<SittingDetected>(sitting));

            // Notify if sensor data suggests robot is standing up... TODO: likely requies greater collection of sensor
            // data
            // StandingDetected standing;
            //    ...
            // emit(std::make_unique<StandingDetected>(standing));

            // Notify if sensor data suggests robot is begining to fall...
            FallingDetected falling(fallingScaleX, fallingScaleY, 0.0);
            emit(std::make_unique<FallingDetected>(falling));
        });
    }
    /*=======================================================================================================*/
    //      INITIALISATION METHOD: Configuration
    /*=======================================================================================================*/
    void PostureRecogniser::configure(const YAML::Node& config) {
        auto& debug            = config["debugging"];
        DEBUG                  = debug["enabled"].as<bool>();
        emitAccelerometer      = debug["emit_accelerometer"].as<bool>();
        emitGyroscope          = debug["emit_gyroscope"].as<bool>();
        emitFallingScaleFactor = debug["emit_falling_scale_factor"].as<bool>();
    }
}  // namespace input
}  // namespace module
