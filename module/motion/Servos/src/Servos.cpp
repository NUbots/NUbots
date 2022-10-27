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
    using message::motion::ArmID;
    using message::motion::Head;
    using message::motion::HeadID;
    using message::motion::HeadPitch;
    using message::motion::HeadYaw;
    using message::motion::LeftAnklePitch;
    using message::motion::LeftAnkleRoll;
    using message::motion::LeftArm;
    using message::motion::LeftElbow;
    using message::motion::LeftHipPitch;
    using message::motion::LeftHipRoll;
    using message::motion::LeftHipYaw;
    using message::motion::LeftKnee;
    using message::motion::LeftLeg;
    using message::motion::LeftShoulderPitch;
    using message::motion::LeftShoulderRoll;
    using message::motion::LegID;
    using message::motion::RightAnklePitch;
    using message::motion::RightAnkleRoll;
    using message::motion::RightArm;
    using message::motion::RightElbow;
    using message::motion::RightHipPitch;
    using message::motion::RightHipRoll;
    using message::motion::RightHipYaw;
    using message::motion::RightKnee;
    using message::motion::RightLeg;
    using message::motion::RightShoulderPitch;
    using message::motion::RightShoulderRoll;
    using utility::input::ServoID;

    Servos::Servos(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Servos.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Servos.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<LeftLeg>,
           Needs<LeftHipYaw>,
           Needs<LeftHipRoll>,
           Needs<LeftHipPitch>,
           Needs<LeftKnee>,
           Needs<LeftAnklePitch>,
           Needs<LeftAnkleRoll>>()
            .then([this](const LeftLeg& leg,
                         const RunInfo& info,
                         const Uses<LeftHipYaw>& lhy,
                         const Uses<LeftHipRoll>& lhr,
                         const Uses<LeftHipPitch>& lhp,
                         const Uses<LeftKnee>& lk,
                         const Uses<LeftAnklePitch>& lap,
                         const Uses<LeftAnkleRoll>& lar) {
                // This is done when all of the servos are done, so check them all
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    if (lhy.done && lhr.done && lhp.done && lk.done && lap.done && lar.done) {
                        emit<Task>(std::make_unique<Done>());
                        return;
                    }
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // Emit tasks for each servo
                emit<Task>(std::make_unique<LeftHipYaw>(leg.servos[LegID::HIP_YAW]));
                emit<Task>(std::make_unique<LeftHipRoll>(leg.servos[LegID::HIP_ROLL]));
                emit<Task>(std::make_unique<LeftHipPitch>(leg.servos[LegID::HIP_PITCH]));
                emit<Task>(std::make_unique<LeftKnee>(leg.servos[LegID::KNEE]));
                emit<Task>(std::make_unique<LeftAnklePitch>(leg.servos[LegID::ANKLE_PITCH]));
                emit<Task>(std::make_unique<LeftAnkleRoll>(leg.servos[LegID::ANKLE_ROLL]));
            });

        on<Provide<RightLeg>,
           Needs<RightHipYaw>,
           Needs<RightHipRoll>,
           Needs<RightHipPitch>,
           Needs<RightKnee>,
           Needs<RightAnklePitch>,
           Needs<RightAnkleRoll>>()
            .then([this](const RightLeg& leg,
                         const RunInfo& info,
                         const Uses<RightHipYaw>& rhy,
                         const Uses<RightHipRoll>& rhr,
                         const Uses<RightHipPitch>& rhp,
                         const Uses<RightKnee>& rk,
                         const Uses<RightAnklePitch>& rap,
                         const Uses<RightAnkleRoll>& rar) {
                // This is done when all of the servos are done, so check them all
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    if (rhy.done && rhr.done && rhp.done && rk.done && rap.done && rar.done) {
                        emit<Task>(std::make_unique<Done>());
                        return;
                    }
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // Emit tasks for each servo
                emit<Task>(std::make_unique<RightHipYaw>(leg.servos[LegID::HIP_YAW]));
                emit<Task>(std::make_unique<RightHipRoll>(leg.servos[LegID::HIP_ROLL]));
                emit<Task>(std::make_unique<RightHipPitch>(leg.servos[LegID::HIP_PITCH]));
                emit<Task>(std::make_unique<RightKnee>(leg.servos[LegID::KNEE]));
                emit<Task>(std::make_unique<RightAnklePitch>(leg.servos[LegID::ANKLE_PITCH]));
                emit<Task>(std::make_unique<RightAnkleRoll>(leg.servos[LegID::ANKLE_ROLL]));
            });

        on<Provide<LeftArm>, Needs<LeftShoulderPitch>, Needs<LeftShoulderRoll>, Needs<LeftElbow>>().then(
            [this](const LeftArm& arm,
                   const RunInfo& info,
                   const Uses<LeftShoulderPitch>& lsp,
                   const Uses<LeftShoulderRoll>& lsr,
                   const Uses<LeftElbow>& le) {
                // This is done when all of the servos are done, so check them all
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    if (lsp.done && lsr.done && le.done) {
                        emit<Task>(std::make_unique<Done>());
                        return;
                    }
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // Emit tasks for each servo
                emit<Task>(std::make_unique<LeftShoulderPitch>(arm.servos[ArmID::SHOULDER_PITCH]));
                emit<Task>(std::make_unique<LeftShoulderRoll>(arm.servos[ArmID::SHOULDER_ROLL]));
                emit<Task>(std::make_unique<LeftElbow>(arm.servos[ArmID::ELBOW]));
            });

        on<Provide<RightArm>, Needs<RightShoulderPitch>, Needs<RightShoulderRoll>, Needs<RightElbow>>().then(
            [this](const RightArm& arm,
                   const RunInfo& info,
                   const Uses<RightShoulderPitch>& rsp,
                   const Uses<RightShoulderRoll>& rsr,
                   const Uses<RightElbow>& re) {
                // This is done when all of the servos are done, so check them all
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    if (rsp.done && rsr.done && re.done) {
                        emit<Task>(std::make_unique<Done>());
                        return;
                    }
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // Emit tasks for each servo
                emit<Task>(std::make_unique<RightShoulderPitch>(arm.servos[ArmID::SHOULDER_PITCH]));
                emit<Task>(std::make_unique<RightShoulderRoll>(arm.servos[ArmID::SHOULDER_ROLL]));
                emit<Task>(std::make_unique<RightElbow>(arm.servos[ArmID::ELBOW]));
            });

        on<Provide<Head>, Needs<HeadYaw>, Needs<HeadPitch>>().then(
            [this](const Head& head, const RunInfo& info, const Uses<HeadYaw>& hy, const Uses<HeadPitch>& hp) {
                // This is done when all of the servos are done, so check them all
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    if (hy.done && hp.done) {
                        emit<Task>(std::make_unique<Done>());
                        return;
                    }
                    emit<Task>(std::make_unique<Idle>());
                    return;
                }

                // Emit tasks for each servo
                emit<Task>(std::make_unique<HeadYaw>(head.servos[HeadID::YAW]));
                emit<Task>(std::make_unique<HeadPitch>(head.servos[HeadID::PITCH]));
            });


        add_servo_provider<RightShoulderPitch, ServoID::Value::R_SHOULDER_PITCH>();
        add_servo_provider<LeftShoulderPitch, ServoID::Value::L_SHOULDER_PITCH>();
        add_servo_provider<RightShoulderRoll, ServoID::Value::R_SHOULDER_ROLL>();
        add_servo_provider<LeftShoulderRoll, ServoID::Value::L_SHOULDER_ROLL>();
        add_servo_provider<RightElbow, ServoID::Value::R_ELBOW>();
        add_servo_provider<LeftElbow, ServoID::Value::L_ELBOW>();
        add_servo_provider<RightHipYaw, ServoID::Value::R_HIP_YAW>();
        add_servo_provider<LeftHipYaw, ServoID::Value::L_HIP_YAW>();
        add_servo_provider<RightHipRoll, ServoID::Value::R_HIP_ROLL>();
        add_servo_provider<LeftHipRoll, ServoID::Value::L_HIP_ROLL>();
        add_servo_provider<RightHipPitch, ServoID::Value::R_HIP_PITCH>();
        add_servo_provider<LeftHipPitch, ServoID::Value::L_HIP_PITCH>();
        add_servo_provider<RightKnee, ServoID::Value::R_KNEE>();
        add_servo_provider<LeftKnee, ServoID::Value::L_KNEE>();
        add_servo_provider<RightAnklePitch, ServoID::Value::R_ANKLE_PITCH>();
        add_servo_provider<LeftAnklePitch, ServoID::Value::L_ANKLE_PITCH>();
        add_servo_provider<RightAnkleRoll, ServoID::Value::R_ANKLE_ROLL>();
        add_servo_provider<LeftAnkleRoll, ServoID::Value::L_ANKLE_ROLL>();
        add_servo_provider<HeadYaw, ServoID::Value::HEAD_YAW>();
        add_servo_provider<HeadPitch, ServoID::Value::HEAD_PITCH>();
    }

}  // namespace module::motion
