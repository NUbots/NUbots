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

#ifndef MODULE_MOTION_KINEMATICSCONFIGURATION_HPP
#define MODULE_MOTION_KINEMATICSCONFIGURATION_HPP

#include <Eigen/Core>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"

#include "message/motion/KinematicsModel.hpp"

namespace module::motion {

    class KinematicsConfiguration : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the KinematicsConfiguration reactor.
        explicit KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment);

    private:
        static void configure(message::motion::KinematicsModel& model, const ::extension::Configuration& objNugusModel);
        static void configureLeg(message::motion::KinematicsModel& model, const YAML::Node& objLeg);
        static void configureHead(message::motion::KinematicsModel& model, const YAML::Node& objHead);
        static void configureArm(message::motion::KinematicsModel& model, const YAML::Node& objArm);
        static void configureMassModel(message::motion::KinematicsModel& model, const YAML::Node& objMassModel);
        static void configureTensorModel(message::motion::KinematicsModel& model, const YAML::Node& objTensorModel);
    };
}  // namespace module::motion

#endif  // MODULE_MOTION_KINEMATICSCONFIGURATION_HPP
