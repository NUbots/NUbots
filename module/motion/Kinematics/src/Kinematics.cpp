#include "Kinematics.hpp"

#include "extension/Configuration.hpp"

#include "utility/motion/InverseKinematics.hpp"

namespace module::motion {

    using extension::Configuration;

    Kinematics::Kinematics(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("Kinematics.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Kinematics.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        /// @brief Save the kinematics model for use in the inverse kinematics function
        on<Startup, Trigger<KinematicsModel>>().then("Update Kinematics Model",
                                                     [this](const KinematicsModel& model) { kinematicsModel = model; });

        /// @brief Calculates left leg kinematics and makes a task for the LeftLeg servos
        on<Provides<LeftLegIK>, Needs<LeftLeg>, Trigger<Sensors>>().then(
            [this](const LeftLegIK& leg_ik, const Sensors& /* sensors */) {
                // If the time to reach the position is over, then stop requesting the position
                if (NUClear::clock::now() >= leg_ik.time) {
                    emit<Task>(std::make_unique<Done>());
                }
                auto servos = std::make_unique<LeftLeg>();
                auto joints = calculateLegJoints<float>(kinematicsModel, leg_ik.Htl, LimbID::LEFT_LEG);

                // The order of the servos in LeftLegIK and LeftLeg should be LeftLeg.ID
                for (int i = 0; i < joints.size(); i++) {
                    servos->servos.push_back(leg_ik.time, joints[i].second, leg_ik.servos[i]);
                }
                log<NUClear::DEBUG>("Emitting left leg request from a left leg IK provider.");
                emit<Task>(servos);
            });

        /// @brief Calculates right leg kinematics and makes a task for the RightLeg servos
        on<Provides<RightLegIK>, Needs<RightLeg>, Trigger<Sensors>>().then(
            [this](const RightLegIK leg_ik, const Sensors& /* sensors */) {
                // If the time to reach the position is over, then stop requesting the position
                if (NUClear::clock::now() >= leg_ik.time) {
                    emit<Task>(std::make_unique<Done>());
                }
                auto servos = std::make_unique<LeftLeg>();
                auto joints = calculateLegJoints<float>(kinematicsModel, leg_ik.Htr, LimbID::RIGHT_LEG);

                // The order of the servos in RightLegIK and RightLeg should be RightLeg.ID
                for (int i = 0; i < joints.size(); i++) {
                    servos->servos.push_back(leg_ik.time, joints[i].second, leg_ik.servos[i]);
                }
                log<NUClear::DEBUG>("Emitting right leg request from a right leg IK provider.");
                emit<Task>(servos);
            });

        /// @brief Calculates head kinematics and makes a task for the Head servos
        on<Provides<HeadIK>, Needs<Head>, Trigger<Sensors>>().then(
            [this](const HeadIK head_ik, const Sensors& /* sensors */) {
                // If the time to reach the position is over, then stop requesting the position
                if (NUClear::clock::now() >= leg_ik.time) {
                    emit<Task>(std::make_unique<Done>());
                }
                auto servos = std::make_unique<Head>();
                auto joints = calculateHeadJoints<float>(head_ik.uPCt);

                // The order of the servos in HeadIK and Head should be Head.ID (yaw, pitch)
                for (int i = 0; i < joints.size(); i++) {
                    servos->servos.push_back(head_ik.time, joints[i].second, head_ik.servos[i]);
                }
                log<NUClear::DEBUG>("Emitting head request from a head IK provider.");
                emit<Task>(servos);
            });
    }

}  // namespace module::motion
