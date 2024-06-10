/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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

#include "Servos.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/Servos.hpp"

namespace module::actuation {

    using extension::Configuration;
    using message::actuation::Arms;
    using message::actuation::ArmsSequence;
    using message::actuation::Body;
    using message::actuation::BodySequence;
    using message::actuation::Head;
    using message::actuation::HeadPitch;
    using message::actuation::HeadSequence;
    using message::actuation::HeadYaw;
    using message::actuation::LeftAnklePitch;
    using message::actuation::LeftAnkleRoll;
    using message::actuation::LeftArm;
    using message::actuation::LeftArmSequence;
    using message::actuation::LeftElbow;
    using message::actuation::LeftHipPitch;
    using message::actuation::LeftHipRoll;
    using message::actuation::LeftHipYaw;
    using message::actuation::LeftKnee;
    using message::actuation::LeftLeg;
    using message::actuation::LeftLegSequence;
    using message::actuation::LeftShoulderPitch;
    using message::actuation::LeftShoulderRoll;
    using message::actuation::Legs;
    using message::actuation::LegsSequence;
    using message::actuation::Limbs;
    using message::actuation::LimbsSequence;
    using message::actuation::RightAnklePitch;
    using message::actuation::RightAnkleRoll;
    using message::actuation::RightArm;
    using message::actuation::RightArmSequence;
    using message::actuation::RightElbow;
    using message::actuation::RightHipPitch;
    using message::actuation::RightHipRoll;
    using message::actuation::RightHipYaw;
    using message::actuation::RightKnee;
    using message::actuation::RightLeg;
    using message::actuation::RightLegSequence;
    using message::actuation::RightShoulderPitch;
    using message::actuation::RightShoulderRoll;
    using message::actuation::ServoID;

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

}  // namespace module::actuation
