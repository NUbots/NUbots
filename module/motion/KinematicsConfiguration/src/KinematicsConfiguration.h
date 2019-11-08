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

#ifndef MODULE_MOTION_KINEMATICSCONFIGURATION_H
#define MODULE_MOTION_KINEMATICSCONFIGURATION_H

#include <yaml-cpp/yaml.h>

#include <armadillo>
#include <nuclear>

#include "extension/Configuration.h"
#include "message/motion/KinematicsModel.h"

namespace module {
namespace motion {

    class KinematicsConfiguration : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the KinematicsConfiguration reactor.
        explicit KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment);

    private:
        void configure(message::motion::KinematicsModel& model, const ::extension::Configuration& objDarwinModel);
        void configureLeg(message::motion::KinematicsModel& model, const YAML::Node& objLeg);
        void configureHead(message::motion::KinematicsModel& model, const YAML::Node& objHead);
        void configureArm(message::motion::KinematicsModel& model, const YAML::Node& objArm);
        void configureMassModel(message::motion::KinematicsModel& model, const YAML::Node& objMassModel);
        void configureTensorModel(message::motion::KinematicsModel& model, const YAML::Node& objTensorModel);
    };
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_KINEMATICSCONFIGURATION_H
