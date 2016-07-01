#include "LegLoadsLogger.h"

#include "message/support/Configuration.h"
#include "message/input/Sensors.h"
#include "message/input/ServoID.h"

namespace module {
namespace support {
namespace logging {

    using message::support::Configuration;
    using message::input::Sensors;
    using message::input::ServoID;

    LegLoadsLogger::LegLoadsLogger(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), logFile(), logFilePath("NON_EXISTENT_FILE.CSV"), logFileHeader("LEG LOADS LOGGER") {

        on<Configuration>("LegLoadsLogger.yaml").then("Leg Loads Logger Configuration", [this] (const Configuration& config) {
            logFilePath   = config["log_file"].as<std::string>("NON_EXISTENT_FILE.CSV");
            logFileHeader = config["log_header"].as<std::string>("LEG LOADS LOGGER");
        });

        on<Startup>().then("Leg Loads Logger Startup", [this] ()
        {
            logFile.open(logFilePath, std::ios::out | std::ios::binary);

            if ((logFile.is_open() == true) && (logFile.good() == true))
            {
                logFile << logFileHeader << std::endl;
                logFile << "RIGHT_FOOT_DISPLACEMENT, LEFT_FOOT_DISPLACEMENT, "
                        << "RIGHT_HIP_PITCH_PRESENT_VELOCITY, RIGHT_HIP_PITCH_LOAD, "
                        << "LEFT_HIP_PITCH_PRESENT_VELOCITY, LEFT_HIP_PITCH_LOAD, "
                        << "RIGHT_KNEE_PRESENT_VELOCITY, RIGHT_KNEE_LOAD, "
                        << "LEFT_KNEE_PRESENT_VELOCITY, LEFT_KNEE_LOAD, "
                        << "RIGHT_ANKLE_PITCH_PRESENT_VELOCITY, RIGHT_ANKLE_PITCH_LOAD, "
                        << "LEFT_ANKLE_PITCH_PRESENT_VELOCITY, LEFT_ANKLE_PITCH_LOAD" << std::endl;
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
            arma::vec3 RightFootDisplacement          = -sensors.forwardKinematics.at(ServoID::R_ANKLE_ROLL).i().translation();
            arma::vec3 LeftFootDisplacement           = -sensors.forwardKinematics.at(ServoID::L_ANKLE_ROLL).i().translation();
            float      RightHipPitchPresentVelocity   =  sensors.servos[int(ServoID::R_HIP_PITCH)].presentVelocity;
            float      RightHipPitchLoad              =  sensors.servos[int(ServoID::R_HIP_PITCH)].load;
            float      LeftHipPitchPresentVelocity    =  sensors.servos[int(ServoID::L_HIP_PITCH)].presentVelocity;
            float      LeftHipPitchLoad               =  sensors.servos[int(ServoID::L_HIP_PITCH)].load;
            float      RightKneePresentVelocity       =  sensors.servos[int(ServoID::R_KNEE)].presentVelocity;
            float      RightKneeLoad                  =  sensors.servos[int(ServoID::R_KNEE)].load;
            float      LeftKneePresentVelocity        =  sensors.servos[int(ServoID::L_KNEE)].presentVelocity;
            float      LeftKneeLoad                   =  sensors.servos[int(ServoID::L_KNEE)].load;
            float      RightAnklePitchPresentVelocity =  sensors.servos[int(ServoID::R_ANKLE_PITCH)].presentVelocity;
            float      RightAnklePitchLoad            =  sensors.servos[int(ServoID::R_ANKLE_PITCH)].load;
            float      LeftAnklePitchPresentVelocity  =  sensors.servos[int(ServoID::L_ANKLE_PITCH)].presentVelocity;
            float      LeftAnklePitchLoad             =  sensors.servos[int(ServoID::L_ANKLE_PITCH)].load;

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
