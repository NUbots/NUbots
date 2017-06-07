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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#include "LegLoadsLogger.h"

#include <Eigen/Core>

#include "extension/Configuration.h"

#include "message/input/Sensors.h"

#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"

namespace module {
namespace support {
namespace logging {

    using extension::Configuration;

    using message::input::Sensors;
    using ServoID = utility::input::ServoID;

    using utility::math::matrix::Transform3D;

    LegLoadsLogger::LegLoadsLogger(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), logFile(), logFilePath("NON_EXISTENT_FILE.CSV") {

        on<Configuration>("LegLoadsLogger.yaml").then("Leg Loads Logger Configuration", [this] (const Configuration& config) {
            //logFilePath = config["log_file"].as<std::string>("NON_EXISTENT_FILE.CSV");
            logFilePath = config["log_file"].as<std::string>();
        });

        on<Startup>().then("Leg Loads Logger Startup", [this] ()
        {
            logFile.open(logFilePath, std::ios::out | std::ios::binary);

            if ((logFile.is_open() == true) && (logFile.good() == true))
            {
                logFile << "RIGHT_FOOT_DISPLACEMENT, LEFT_FOOT_DISPLACEMENT, "
                        << "R_HIP_PITCH_PRESENT_VELOCITY, R_HIP_PITCH_LOAD, "
                        << "L_HIP_PITCH_PRESENT_VELOCITY, L_HIP_PITCH_LOAD, "
                        << "R_KNEE_PRESENT_VELOCITY, R_KNEE_LOAD, "
                        << "L_KNEE_PRESENT_VELOCITY, L_KNEE_LOAD, "
                        << "R_ANKLE_PITCH_PRESENT_VELOCITY, R_ANKLE_PITCH_LOAD, "
                        << "L_ANKLE_PITCH_PRESENT_VELOCITY, L_ANKLE_PITCH_LOAD" << std::endl;
            }

            else
            {
                NUClear::log<NUClear::ERROR>("Failed to open log file '", logFilePath, "'.");
            }
        });

        on<Shutdown>().then("Leg Loads Logger Shutdown", [this] ()
        {
            logFile.close();
        });

        on<Trigger<Sensors>>().then("Leg Loads Logger Sensor Update", [this] (const Sensors& sensors)
        {
            // RightFootDisplacement FK -> RightAnkleRoll -> inverse -> translation -> negate
            // LeftFootDisplacement  FK -> LeftAnkleRoll  -> inverse -> translation -> negate
            Eigen::Vector3d RightFootDisplacement          = -Transform3D(sensors.forwardKinematics.at(ServoID::R_ANKLE_ROLL).inverse()).translation();
            Eigen::Vector3d LeftFootDisplacement           = -Transform3D(sensors.forwardKinematics.at(ServoID::L_ANKLE_ROLL).inverse()).translation();
            float      RightHipPitchPresentVelocity   =  sensors.servo[ServoID::R_HIP_PITCH].presentVelocity;
            float      RightHipPitchLoad              =  sensors.servo[ServoID::R_HIP_PITCH].load;
            float      LeftHipPitchPresentVelocity    =  sensors.servo[ServoID::L_HIP_PITCH].presentVelocity;
            float      LeftHipPitchLoad               =  sensors.servo[ServoID::L_HIP_PITCH].load;
            float      RightKneePresentVelocity       =  sensors.servo[ServoID::R_KNEE].presentVelocity;
            float      RightKneeLoad                  =  sensors.servo[ServoID::R_KNEE].load;
            float      LeftKneePresentVelocity        =  sensors.servo[ServoID::L_KNEE].presentVelocity;
            float      LeftKneeLoad                   =  sensors.servo[ServoID::L_KNEE].load;
            float      RightAnklePitchPresentVelocity =  sensors.servo[ServoID::R_ANKLE_PITCH].presentVelocity;
            float      RightAnklePitchLoad            =  sensors.servo[ServoID::R_ANKLE_PITCH].load;
            float      LeftAnklePitchPresentVelocity  =  sensors.servo[ServoID::L_ANKLE_PITCH].presentVelocity;
            float      LeftAnklePitchLoad             =  sensors.servo[ServoID::L_ANKLE_PITCH].load;

            if ((logFile.is_open() == true) && (logFile.good() == true))
            {
                logFile << RightFootDisplacement.at(2)    << ", " << LeftFootDisplacement.at(2) << ", "
                        << RightHipPitchPresentVelocity   << ", " << RightHipPitchLoad          << ", "
                        << LeftHipPitchPresentVelocity    << ", " << LeftHipPitchLoad           << ", "
                        << RightKneePresentVelocity       << ", " << RightKneeLoad              << ", "
                        << LeftKneePresentVelocity        << ", " << LeftKneeLoad               << ", "
                        << RightAnklePitchPresentVelocity << ", " << RightAnklePitchLoad        << ", "
                        << LeftAnklePitchPresentVelocity  << ", " << LeftAnklePitchLoad         << std::endl;
            }

            else
            {
                NUClear::log<NUClear::ERROR>("Log file '", logFilePath, "' is not open or is not good.");
            }
        });
    }
}
}
}
