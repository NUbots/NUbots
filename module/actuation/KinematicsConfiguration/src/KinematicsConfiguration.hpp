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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_ACTUATION_KINEMATICSCONFIGURATION_HPP
#define MODULE_ACTUATION_KINEMATICSCONFIGURATION_HPP

#include <Eigen/Core>
#include <nuclear>
#include <tinyrobotics/Kinematics.hpp>
#include <tinyrobotics/Parser.hpp>
#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"

namespace module::actuation {

    using namespace tinyrobotics;

    class KinematicsConfiguration : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the KinematicsConfiguration reactor.
        explicit KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Path to the URDF file
            std::string urdf_path;
        } cfg;

        /// @brief Number of actuatable joints in the NUgus robot
        static const int n_joints = 20;

        /// @brief Tinyrobotics model of the NUgus robot
        tinyrobotics::Model<float, n_joints> nugus_model;

        /// @brief Configures the NUclear robot model message with values from config file and tinyrobotics model
        void configure(message::actuation::KinematicsModel& model, const ::extension::Configuration& config);

        /// @brief Configures the legs
        void configure_leg(message::actuation::KinematicsModel& model, const YAML::Node& obj_leg);

        /// @brief Configures the head
        void configure_head(message::actuation::KinematicsModel& model, const YAML::Node& obj_head);

        /// @brief Configures the arms
        void configure_arm(message::actuation::KinematicsModel& model, const YAML::Node& obj_arm);

        /// @brief Configures the mass model
        void configure_mass_model(message::actuation::KinematicsModel& model, const YAML::Node& obj_mass_model);

        /// @brief Configures the tensor model
        void configure_tensor_model(message::actuation::KinematicsModel& model, const YAML::Node& obj_tensor_model);
    };
}  // namespace module::actuation

#endif  // MODULE_ACTUATION_KINEMATICSCONFIGURATION_HPP
