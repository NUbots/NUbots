#include "Kinematics.hpp"

#include <extension/Behaviour.hpp>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/motion/Limbs.hpp"
#include "message/motion/LimbsIK.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/motion/InverseKinematics.hpp"

namespace module::motion {

    using extension::Configuration;
    using message::input::Sensors;
    using message::motion::Head;
    using message::motion::HeadIK;
    using message::motion::KinematicsModel;
    using message::motion::LeftLeg;
    using message::motion::LeftLegIK;
    using message::motion::RightLeg;
    using message::motion::RightLegIK;
    using utility::input::LimbID;
    using utility::motion::kinematics::calculateHeadJoints;
    using utility::motion::kinematics::calculateLegJoints;

    Kinematics::Kinematics(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Kinematics.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Kinematics.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        /// @brief Save the kinematics model for use in the inverse kinematics function
        on<Startup, Trigger<KinematicsModel>>().then("Update Kinematics Model",
                                                     [this](const KinematicsModel& model) { kinematicsModel = model; });

        /// @brief Calculates left leg kinematics and makes a task for the LeftLeg servos
        on<Provide<LeftLegIK>, Needs<LeftLeg>, Trigger<Sensors>>().then([this](const LeftLegIK& leg_ik,
                                                                               const Sensors& /* sensors */) {
            // If the time to reach the position is over, then stop requesting the position
            if (NUClear::clock::now() >= leg_ik.time) {
                emit<Task>(std::make_unique<Done>());
            }
            auto servos = std::make_unique<LeftLeg>();
            auto joints = calculateLegJoints<double>(kinematicsModel, Eigen::Affine3d(leg_ik.Htl), LimbID::LEFT_LEG);

            // The order of the servos in LeftLegIK and LeftLeg should be LeftLeg.ID
            for (long unsigned int i = 0; i < joints.size(); i++) {
                servos->servos.emplace_back(leg_ik.time, joints[i].second, leg_ik.servos[i]);
            }
            log<NUClear::DEBUG>("Emitting left leg request from a left leg IK provider.");
            emit<Task>(servos);
        });

        /// @brief Calculates right leg kinematics and makes a task for the RightLeg servos
        on<Provide<RightLegIK>, Needs<RightLeg>, Trigger<Sensors>>().then([this](const RightLegIK leg_ik,
                                                                                 const Sensors& /* sensors */) {
            // If the time to reach the position is over, then stop requesting the position
            if (NUClear::clock::now() >= leg_ik.time) {
                emit<Task>(std::make_unique<Done>());
            }
            auto servos = std::make_unique<LeftLeg>();
            auto joints = calculateLegJoints<double>(kinematicsModel, Eigen::Affine3d(leg_ik.Htr), LimbID::RIGHT_LEG);

            // The order of the servos in RightLegIK and RightLeg should be RightLeg.ID
            for (long unsigned int i = 0; i < joints.size(); i++) {
                servos->servos.emplace_back(leg_ik.time, joints[i].second, leg_ik.servos[i]);
            }
            log<NUClear::DEBUG>("Emitting right leg request from a right leg IK provider.");
            emit<Task>(servos);
        });

        /// @brief Calculates head kinematics and makes a task for the Head servos
        on<Provide<HeadIK>, Needs<Head>, Trigger<Sensors>>().then(
            [this](const HeadIK head_ik, const Sensors& /* sensors */) {
                // If the time to reach the position is over, then stop requesting the position
                if (NUClear::clock::now() >= head_ik.time) {
                    emit<Task>(std::make_unique<Done>());
                }
                auto servos = std::make_unique<Head>();
                auto joints = calculateHeadJoints<double>(Eigen::Vector3d(head_ik.uPCt));

                // The order of the servos in HeadIK and Head should be Head.ID (yaw, pitch)
                for (long unsigned int i = 0; i < joints.size(); i++) {
                    servos->servos.emplace_back(head_ik.time, joints[i].second, head_ik.servos[i]);
                }
                log<NUClear::DEBUG>("Emitting head request from a head IK provider.");
                emit<Task>(servos);
            });
    }

}  // namespace module::motion
