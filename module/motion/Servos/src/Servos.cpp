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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include "Servos.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/motion/Limbs.hpp"
#include "message/motion/Servos.hpp"

#include "utility/input/ServoID.hpp"

namespace module::motion {

    using extension::Configuration;
    using message::motion::Arms;
    using message::motion::ArmsSequence;
    using message::motion::Body;
    using message::motion::BodySequence;
    using message::motion::Head;
    using message::motion::HeadPitch;
    using message::motion::HeadSequence;
    using message::motion::HeadYaw;
    using message::motion::LeftAnklePitch;
    using message::motion::LeftAnkleRoll;
    using message::motion::LeftArm;
    using message::motion::LeftArmSequence;
    using message::motion::LeftElbow;
    using message::motion::LeftHipPitch;
    using message::motion::LeftHipRoll;
    using message::motion::LeftHipYaw;
    using message::motion::LeftKnee;
    using message::motion::LeftLeg;
    using message::motion::LeftLegSequence;
    using message::motion::LeftShoulderPitch;
    using message::motion::LeftShoulderRoll;
    using message::motion::Legs;
    using message::motion::LegsSequence;
    using message::motion::Limbs;
    using message::motion::LimbsSequence;
    using message::motion::RightAnklePitch;
    using message::motion::RightAnkleRoll;
    using message::motion::RightArm;
    using message::motion::RightArmSequence;
    using message::motion::RightElbow;
    using message::motion::RightHipPitch;
    using message::motion::RightHipRoll;
    using message::motion::RightHipYaw;
    using message::motion::RightKnee;
    using message::motion::RightLeg;
    using message::motion::RightLegSequence;
    using message::motion::RightShoulderPitch;
    using message::motion::RightShoulderRoll;
    using utility::input::ServoID;

    Servos::Servos(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Servos.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Servos.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        // Create providers for each servo
        add_servo_provider<RightShoulderPitch, ServoID::R_SHOULDER_PITCH>();
        add_servo_provider<LeftShoulderPitch, ServoID::L_SHOULDER_PITCH>();
        add_servo_provider<RightShoulderRoll, ServoID::R_SHOULDER_ROLL>();
        add_servo_provider<LeftShoulderRoll, ServoID::L_SHOULDER_ROLL>();
        add_servo_provider<RightElbow, ServoID::R_ELBOW>();
        add_servo_provider<LeftElbow, ServoID::L_ELBOW>();
        add_servo_provider<RightHipYaw, ServoID::R_HIP_YAW>();
        add_servo_provider<LeftHipYaw, ServoID::L_HIP_YAW>();
        add_servo_provider<RightHipRoll, ServoID::R_HIP_ROLL>();
        add_servo_provider<LeftHipRoll, ServoID::L_HIP_ROLL>();
        add_servo_provider<RightHipPitch, ServoID::R_HIP_PITCH>();
        add_servo_provider<LeftHipPitch, ServoID::L_HIP_PITCH>();
        add_servo_provider<RightKnee, ServoID::R_KNEE>();
        add_servo_provider<LeftKnee, ServoID::L_KNEE>();
        add_servo_provider<RightAnklePitch, ServoID::R_ANKLE_PITCH>();
        add_servo_provider<LeftAnklePitch, ServoID::L_ANKLE_PITCH>();
        add_servo_provider<RightAnkleRoll, ServoID::R_ANKLE_ROLL>();
        add_servo_provider<LeftAnkleRoll, ServoID::L_ANKLE_ROLL>();
        add_servo_provider<HeadYaw, ServoID::HEAD_YAW>();
        add_servo_provider<HeadPitch, ServoID::HEAD_PITCH>();

        // Create providers for each limb and the head
        add_group_provider<RightLeg,
                           RightHipYaw,
                           RightHipRoll,
                           RightHipPitch,
                           RightKnee,
                           RightAnklePitch,
                           RightAnkleRoll>();
        add_group_provider<LeftLeg, LeftHipYaw, LeftHipRoll, LeftHipPitch, LeftKnee, LeftAnklePitch, LeftAnkleRoll>();
        add_group_provider<RightArm, RightShoulderPitch, RightShoulderRoll, RightElbow>();
        add_group_provider<LeftArm, LeftShoulderPitch, LeftShoulderRoll, LeftElbow>();
        add_group_provider<Head, HeadYaw, HeadPitch>();

        // Create providers for each limb/head grouping
        add_group_provider<Body,
                           LeftHipYaw,
                           LeftHipRoll,
                           LeftHipPitch,
                           LeftKnee,
                           LeftAnklePitch,
                           LeftAnkleRoll,
                           RightHipYaw,
                           RightHipRoll,
                           RightHipPitch,
                           RightKnee,
                           RightAnklePitch,
                           RightAnkleRoll,
                           RightShoulderPitch,
                           RightShoulderRoll,
                           RightElbow,
                           LeftShoulderPitch,
                           LeftShoulderRoll,
                           LeftElbow,
                           HeadYaw,
                           HeadPitch>();
        add_group_provider<Limbs,
                           LeftHipYaw,
                           LeftHipRoll,
                           LeftHipPitch,
                           LeftKnee,
                           LeftAnklePitch,
                           LeftAnkleRoll,
                           RightHipYaw,
                           RightHipRoll,
                           RightHipPitch,
                           RightKnee,
                           RightAnklePitch,
                           RightAnkleRoll,
                           RightShoulderPitch,
                           RightShoulderRoll,
                           RightElbow,
                           LeftShoulderPitch,
                           LeftShoulderRoll,
                           LeftElbow>();
        add_group_provider<Legs,
                           LeftHipYaw,
                           LeftHipRoll,
                           LeftHipPitch,
                           LeftKnee,
                           LeftAnklePitch,
                           LeftAnkleRoll,
                           RightHipYaw,
                           RightHipRoll,
                           RightHipPitch,
                           RightKnee,
                           RightAnklePitch,
                           RightAnkleRoll>();
        add_group_provider<Arms,
                           RightShoulderPitch,
                           RightShoulderRoll,
                           RightElbow,
                           LeftShoulderPitch,
                           LeftShoulderRoll,
                           LeftElbow>();

        // Sequences of servos
        add_sequence_provider<BodySequence, Body>();
        add_sequence_provider<LimbsSequence, Limbs>();
        add_sequence_provider<RightLegSequence, RightLeg>();
        add_sequence_provider<LeftLegSequence, LeftLeg>();
        add_sequence_provider<LegsSequence, Legs>();
        add_sequence_provider<RightArmSequence, RightArm>();
        add_sequence_provider<LeftArmSequence, LeftArm>();
        add_sequence_provider<ArmsSequence, Arms>();
        add_sequence_provider<HeadSequence, Head>();
    }

}  // namespace module::motion
