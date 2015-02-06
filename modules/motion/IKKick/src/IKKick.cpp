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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "IKKick.h"

#include "messages/support/Configuration.h"
#include "messages/motion/KickCommand.h"
#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"
#include "messages/input/LimbID.h"
#include "messages/behaviour/ServoCommand.h"
#include "messages/behaviour/Action.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/support/yaml_armadillo.h"

namespace modules {
namespace motion {

    using messages::support::Configuration;
    using messages::motion::KickCommand;
    using messages::motion::KickFinished;
    using messages::input::Sensors;
    using messages::input::ServoID;
    using messages::input::LimbID;
    using messages::behaviour::ServoCommand;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;

    using utility::math::matrix::Transform3D;
    using utility::motion::kinematics::calculateLegJoints;
    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::motion::kinematics::DarwinModel;

    struct ExecuteKick{};
    struct FinishKick{};

    IKKick::IKKick(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        on<Trigger<Configuration<IKKick>>>([this] (const Configuration<IKKick>& config){
            KICK_PRIORITY = config["kick_priority"].as<float>();
            EXECUTION_PRIORITY = config["execution_priority"].as<float>();

            emit(std::make_unique<KickCommand>(KickCommand{
                config["target"].as<arma::vec3>(),
                config["direction"].as<arma::vec3>()
            }));
        });


        on<Trigger<KickCommand>, With<Sensors>>([this] (const KickCommand& kickCommand, const Sensors&) {

            // We want to kick!
            updatePriority(KICK_PRIORITY);
        });

        on<Trigger<ExecuteKick>, With<KickCommand>, With<Sensors>>([this] (const ExecuteKick&, const KickCommand& command, const Sensors& sensors) {

            // TODO Work out which of our feet are going to be the support foot
            Transform3D leftFoot = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFoot = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            log("Got a new kick!");
            log("Target:", "x:", command.target[0], "y:", command.target[1], "z:", command.target[2]);
            log("Direction:", "x:", command.direction[0], "y:", command.direction[1], "z:", command.direction[2]);

            // TODO Store the target kick vector position relative to the support foot

            // TODO store the support foot

            // Enable our kick pather
            updater.enable();

            updatePriority(EXECUTION_PRIORITY);
        });

        updater = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>, With<Sensors>, Options<Single>>([this](const time_t&, const Sensors& sensors) {

            float gain = 80;
            float torque = 100;

            // Get our foot positions
            Transform3D leftFootTorso = sensors.forwardKinematics.find(ServoID::L_ANKLE_ROLL)->second;
            Transform3D rightFootTorso = sensors.forwardKinematics.find(ServoID::R_ANKLE_ROLL)->second;

            // TODO We're always finshed kicking because we never start :(
            updatePriority(0);

            // If our feet are at the target then stop
            // if(feetatposition) {
                // emit(std::make_unique<FinishKick>());
            // }
            //else {
                // Do a series of transforms and whatnot to put leftFootTorso and RightFootTorso where you want them to be in 1/90th of a second!
                // TODO Move everything towards where it needs to be
            // }

            // Move our feet towards our target
            // auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);
            // auto waypoints = std::make_unique<std::vector<ServoCommand>>();

            // waypoints->reserve(16);
            // time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

            // for (auto& joint : joints) {
            //     waypoints->push_back({ id, time, joint.first, joint.second, gain, torque });
            // }

            // emit(std::move(waypoints));
        });

        on<Trigger<FinishKick>>([this] (const FinishKick&) {
            emit(std::move(std::make_unique<KickFinished>()));
            updater.disable();
            updatePriority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "IK Kick",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM }) },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<ExecuteKick>());
            },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<FinishKick>());
            },
            [this] (const std::set<ServoID>&) {
            }
        }));

            // Listen for kickcommands

            // When we get one ask for prioirty to kick

            // When we get the priority to kick

            // Run a loop that updates our foot position and body position


    }

    void IKKick::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
    }


}
}

