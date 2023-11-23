/*
 * MIT License
 *
 * Copyright (c) 2015 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULE_ACTUATION_KINEMATICSCONFIGURATION_HPP
#define MODULE_ACTUATION_KINEMATICSCONFIGURATION_HPP

#include <Eigen/Core>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"

namespace module::actuation {

    class KinematicsConfiguration : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the KinematicsConfiguration reactor.
        explicit KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment);

    private:
        static void configure(message::actuation::KinematicsModel& model,
                              const ::extension::Configuration& objNugusModel);
        static void configureLeg(message::actuation::KinematicsModel& model, const YAML::Node& objLeg);
        static void configureHead(message::actuation::KinematicsModel& model, const YAML::Node& objHead);
        static void configureArm(message::actuation::KinematicsModel& model, const YAML::Node& objArm);
        static void configureMassModel(message::actuation::KinematicsModel& model, const YAML::Node& objMassModel);
        static void configureTensorModel(message::actuation::KinematicsModel& model, const YAML::Node& objTensorModel);
    };
}  // namespace module::actuation

#endif  // MODULE_ACTUATION_KINEMATICSCONFIGURATION_HPP
