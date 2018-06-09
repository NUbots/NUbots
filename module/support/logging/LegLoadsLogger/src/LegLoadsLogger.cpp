#include "LegLoadsLogger.h"

#include "extension/Configuration.h"

#include "message/input/Sensors.h"

#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace support {
    namespace logging {

        using extension::Configuration;

        using message::input::Sensors;
        using ServoID = utility::input::ServoID;

        using utility::math::matrix::Transform3D;

        LegLoadsLogger::LegLoadsLogger(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), logFile(), logFilePath("NON_EXISTENT_FILE.CSV") {

            on<Configuration>("LegLoadsLogger.yaml")
                .then("Leg Loads Logger Configuration",
                      [this](const Configuration& config) { logFilePath = config["log_file"].as<std::string>(); });

            on<Startup>().then("Leg Loads Logger Startup", [this]() {
                logFile.open(logFilePath, std::ios::out | std::ios::binary);

                if ((logFile.is_open() == true) && (logFile.good() == true)) {
                    logFile << "RIGHT_FOOT_DISPLACEMENT,LEFT_FOOT_DISPLACEMENT,"
                            << "R_HIP_PITCH_PRESENT_VELOCITY,R_HIP_PITCH_LOAD,"
                            << "L_HIP_PITCH_PRESENT_VELOCITY,L_HIP_PITCH_LOAD,"
                            << "R_KNEE_PRESENT_VELOCITY,R_KNEE_LOAD,"
                            << "L_KNEE_PRESENT_VELOCITY,L_KNEE_LOAD,"
                            << "R_ANKLE_PITCH_PRESENT_VELOCITY,R_ANKLE_PITCH_LOAD,"
                            << "L_ANKLE_PITCH_PRESENT_VELOCITY,L_ANKLE_PITCH_LOAD" << std::endl;
                }

                else {
                    NUClear::log<NUClear::ERROR>("Failed to open log file '", logFilePath, "'.");
                }
            });

            on<Shutdown>().then("Leg Loads Logger Shutdown", [this]() { logFile.close(); });

            on<Trigger<Sensors>>().then("Leg Loads Logger Sensor Update", [this](const Sensors& sensors) {
                // RightFootDisplacement FK -> RightAnkleRoll -> inverse -> translation -> negate
                // LeftFootDisplacement  FK -> LeftAnkleRoll  -> inverse -> translation -> negate
                arma::vec3 RightFootDisplacement =
                    -Transform3D(convert<double, 4, 4>(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL].inverse()))
                         .translation();
                arma::vec3 LeftFootDisplacement =
                    -Transform3D(convert<double, 4, 4>(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL].inverse()))
                         .translation();
                float RightHipPitchPresentVelocity   = sensors.servo[ServoID::R_HIP_PITCH].presentVelocity;
                float RightHipPitchLoad              = sensors.servo[ServoID::R_HIP_PITCH].load;
                float LeftHipPitchPresentVelocity    = sensors.servo[ServoID::L_HIP_PITCH].presentVelocity;
                float LeftHipPitchLoad               = sensors.servo[ServoID::L_HIP_PITCH].load;
                float RightKneePresentVelocity       = sensors.servo[ServoID::R_KNEE].presentVelocity;
                float RightKneeLoad                  = sensors.servo[ServoID::R_KNEE].load;
                float LeftKneePresentVelocity        = sensors.servo[ServoID::L_KNEE].presentVelocity;
                float LeftKneeLoad                   = sensors.servo[ServoID::L_KNEE].load;
                float RightAnklePitchPresentVelocity = sensors.servo[ServoID::R_ANKLE_PITCH].presentVelocity;
                float RightAnklePitchLoad            = sensors.servo[ServoID::R_ANKLE_PITCH].load;
                float LeftAnklePitchPresentVelocity  = sensors.servo[ServoID::L_ANKLE_PITCH].presentVelocity;
                float LeftAnklePitchLoad             = sensors.servo[ServoID::L_ANKLE_PITCH].load;

                if ((logFile.is_open() == true) && (logFile.good() == true)) {
                    logFile << RightFootDisplacement.at(2) << "," << LeftFootDisplacement.at(2) << ","
                            << RightHipPitchPresentVelocity << "," << RightHipPitchLoad << ","
                            << LeftHipPitchPresentVelocity << "," << LeftHipPitchLoad << "," << RightKneePresentVelocity
                            << "," << RightKneeLoad << "," << LeftKneePresentVelocity << "," << LeftKneeLoad << ","
                            << RightAnklePitchPresentVelocity << "," << RightAnklePitchLoad << ","
                            << LeftAnklePitchPresentVelocity << "," << LeftAnklePitchLoad << std::endl;
                }

                else {
                    NUClear::log<NUClear::ERROR>("Log file '", logFilePath, "' is not open or is not good.");
                }
            });
        }
    }  // namespace logging
}  // namespace support
}  // namespace module
