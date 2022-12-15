#include "Kinematics.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/input/LimbID.hpp"

namespace module::actuation {

    using extension::Configuration;
    using message::actuation::Head;
    using message::actuation::HeadIK;
    using message::actuation::KinematicsModel;
    using message::actuation::LeftLeg;
    using message::actuation::LeftLegIK;
    using message::actuation::RightLeg;
    using message::actuation::RightLegIK;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using utility::actuation::kinematics::calculateHeadJoints;
    using utility::actuation::kinematics::calculateLegJoints;
    using utility::input::LimbID;

    Kinematics::Kinematics(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Kinematics.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Kinematics.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        /// @brief Calculates left leg kinematics and makes a task for the LeftLeg servos
        on<Provide<LeftLegIK>, With<KinematicsModel>, Needs<LeftLeg>>().then(
            [this](const LeftLegIK& leg_ik, const RunInfo& info, const KinematicsModel& kinematics_model) {
                // If the leg is done moving, then IK is done
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    emit<Task>(std::make_unique<Done>());
                    return;
                }

                // Calculate the joint positions with IK
                auto servos = std::make_unique<LeftLeg>();
                auto joints =
                    calculateLegJoints<double>(kinematics_model, Eigen::Isometry3d(leg_ik.Htl), LimbID::LEFT_LEG);

                for (const auto& joint : joints) {
                    servos->servos[joint.first] =
                        ServoCommand(leg_ik.time, joint.second, leg_ik.servos.at(joint.first));
                }

                emit<Task>(servos);
            });

        /// @brief Calculates right leg kinematics and makes a task for the RightLeg servos
        on<Provide<RightLegIK>, With<KinematicsModel>, Needs<RightLeg>>().then(
            [this](const RightLegIK& leg_ik, const RunInfo& info, const KinematicsModel& kinematics_model) {
                // If the leg is done moving, then IK is done
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    emit<Task>(std::make_unique<Done>());
                    return;
                }

                // Calculate the joint positions with IK
                auto servos = std::make_unique<RightLeg>();
                auto joints =
                    calculateLegJoints<double>(kinematics_model, Eigen::Isometry3d(leg_ik.Htr), LimbID::RIGHT_LEG);

                for (const auto& joint : joints) {
                    servos->servos[joint.first] =
                        ServoCommand(leg_ik.time, joint.second, leg_ik.servos.at(joint.first));
                }

                emit<Task>(servos);
            });

        /// @brief Calculates head kinematics and makes a task for the Head servos
        on<Provide<HeadIK>, Needs<Head>>().then([this](const HeadIK& head_ik, const RunInfo& info) {
            // If the head is done moving, then IK is done
            if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                emit<Task>(std::make_unique<Done>());
                return;
            }

            // Calculate the joint positions with IK
            auto servos = std::make_unique<Head>();
            auto joints = calculateHeadJoints<double>(Eigen::Vector3d(head_ik.uPCt));

            for (const auto& joint : joints) {
                servos->servos[joint.first] = ServoCommand(head_ik.time, joint.second, head_ik.servos.at(joint.first));
            }

            emit<Task>(servos);
        });
    }

}  // namespace module::actuation
