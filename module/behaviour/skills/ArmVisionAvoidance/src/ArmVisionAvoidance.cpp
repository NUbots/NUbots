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

#include "ArmVisionAvoidance.h"

#include "message/input/ServoID.h"
#include "message/input/LimbID.h"
#include "message/behaviour/Action.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/support/Configuration.h"

#include "utility/math/coordinates.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace behaviour {
namespace skills {

    using message::input::LimbID;
    using message::input::ServoID;
    using message::input::Sensors;
    using message::behaviour::ServoCommand;
    using message::behaviour::RegisterAction;
    using message::support::Configuration;
    using utility::support::Expression;

    ArmVisionAvoidance::ArmVisionAvoidance(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

        on<Configuration>("ArmVisionAvoidance.yaml").then([this] (const Configuration& config) {
            // The limits on how far the head can be turned before the corresponding arm starts to obscure vision.
            headYawLimit[0] = config["head_limits"]["yaw"]["min"].as<Expression>();
            headYawLimit[1] = config["head_limits"]["yaw"]["max"].as<Expression>();
            headPitchLimit  = config["head_limits"]["pitch"].as<Expression>();

            // Joint gains and torques.
            gain   = config["gain"].as<float>();
            torque = config["torque"].as<float>();
        });

        updateHandle = on<Trigger<Sensors>, Single, Priority::HIGH>().then("Arm Vision Avoidance - Update Arm Position",
                [this] (const Sensors& sensors) {
            // Check to see if we are looking over one of our shoulders.
            // If we are then we need to add an offset to the arms shoulder and elbow pitches
            // so that the arm is not obscuring our vision.
            auto headTransform = sensors.forwardKinematics.find(ServoID::HEAD_PITCH)->second;
            auto headSpherical = utility::math::coordinates::cartesianToSpherical({headTransform(0, 0), headTransform(1, 0), headTransform(2, 0)});

            auto headYaw   = headSpherical[1];
//            auto headPitch = headSpherical[2];

            // Get the current position of the arm servos.
            float leftShoulderRoll   = sensors.servos.at(static_cast<int>(ServoID::L_SHOULDER_ROLL)).presentPosition;
            float leftShoulderPitch  = sensors.servos.at(static_cast<int>(ServoID::L_SHOULDER_PITCH)).presentPosition;
            float leftElbowPitch     = sensors.servos.at(static_cast<int>(ServoID::L_ELBOW)).presentPosition;
            float rightShoulderRoll  = sensors.servos.at(static_cast<int>(ServoID::R_SHOULDER_ROLL)).presentPosition;
            float rightShoulderPitch = sensors.servos.at(static_cast<int>(ServoID::R_SHOULDER_PITCH)).presentPosition;
            float rightElbowPitch    = sensors.servos.at(static_cast<int>(ServoID::R_ELBOW)).presentPosition;

            if (headYaw > headYawLimit[1]) {        // Looking behind left shoulder.
                leftShoulderPitch   =  headYaw - (M_PI / 2); // (headYawLimit[1] + (M_PI / 2)) - headYaw;
                leftElbowPitch      = -((M_PI / 2) + ((headYawLimit[1] - headYaw) / 2));

                rightShoulderPitch  =  M_PI / 2;
                rightElbowPitch     = -M_PI / 2;
            }

            else if (headYaw > 0) {                 // Looking left.
                leftShoulderPitch   =   (M_PI / 2) + headYaw;
                leftElbowPitch      = -((M_PI / 2) + (headYaw / 2));

                rightShoulderPitch  =  M_PI / 2;
                rightElbowPitch     = -M_PI / 2;
            }

            else if (headYaw < -headYawLimit[1]) {  // Looking behind right shoulder.
                leftShoulderPitch  =  M_PI / 2;
                leftElbowPitch     = -M_PI / 2;

                rightShoulderPitch =  (-headYaw) - (M_PI / 2); // (headYawLimit[1] + (M_PI / 2)) - (-headYaw);
                rightElbowPitch    = -((M_PI / 2) + ((headYawLimit[1] - (-headYaw)) / 2));
            }

            else if (headYaw < 0) {                 // Looking right.
                leftShoulderPitch  =  M_PI / 2;
                leftElbowPitch     = -M_PI / 2;

                rightShoulderPitch =   (M_PI / 2) + (-headYaw);
                rightElbowPitch    = -((M_PI / 2) + ((-headYaw) / 2));
            }

            else {                                  // Looking forward.
                leftShoulderPitch  =  M_PI / 2;
                leftElbowPitch     = -M_PI / 2;

                rightShoulderPitch  =  M_PI / 2;
                rightElbowPitch     = -M_PI / 2;
            }

            // Update the servo positions.
            auto waypoints = std::make_unique<std::vector<ServoCommand>>();
            waypoints->reserve(6);

            //NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::seconds(1);
            NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);
            waypoints->push_back({subsumptionId, time, ServoID::R_SHOULDER_ROLL,  float(rightShoulderRoll),  gain, torque});
            waypoints->push_back({subsumptionId, time, ServoID::R_SHOULDER_PITCH, float(rightShoulderPitch), gain, torque});
            waypoints->push_back({subsumptionId, time, ServoID::R_ELBOW,          float(rightElbowPitch),    gain, torque});
            waypoints->push_back({subsumptionId, time, ServoID::L_SHOULDER_ROLL,  float(leftShoulderRoll),   gain, torque});
            waypoints->push_back({subsumptionId, time, ServoID::L_SHOULDER_PITCH, float(leftShoulderPitch),  gain, torque});
            waypoints->push_back({subsumptionId, time, ServoID::L_ELBOW,          float(leftElbowPitch),     gain, torque});

            emit(std::move(waypoints));
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            subsumptionId,
            "ArmVisionAvoidance",
            { std::pair<float, std::set<LimbID>>(20.0, { LimbID::LEFT_ARM, LimbID::RIGHT_ARM }) },
            [this] (const std::set<LimbID>&) { // Arm control gained
                updateHandle.enable();
            },
            [this] (const std::set<LimbID>&) { // Arm control lost
                updateHandle.disable();
            },
            [this] (const std::set<ServoID>& ) { } // Servos reached target
        }));
    }
}
}
}
