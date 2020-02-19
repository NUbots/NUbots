/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_SUPPORT_OPTIMISATION_WALK_OPTIMISER_H
#define MODULES_SUPPORT_OPTIMISATION_WALK_OPTIMISER_H

#include <Eigen/Core>
#include <nuclear>

#include "extension/Configuration.h"
#include "message/behaviour/FixedWalkCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/GetupCommand.h"
#include "running_stat.hpp"

namespace module {
namespace support {
    namespace optimisation {

        struct OptimiseWalkCommand {};
        struct OptimisationComplete {};


        class FitnessData {
        public:
            uint numberOfGetups = 0;
            running_stat<double> tilt;
            bool recording;
            double popFitness();
            void update(const message::input::Sensors& sensors);
            void recordGetup();
            void getupFinished();
        };

        class WalkOptimiser : public NUClear::Reactor {
        private:
            message::behaviour::FixedWalkCommand walk_command;
            std::vector<std::string> parameter_names;
            Eigen::VectorXd parameter_sigmas;
            Eigen::VectorXd fitnesses;

            unsigned int currentSample;
            Eigen::MatrixXd samples;
            int number_of_samples;

            unsigned int getup_cancel_trial_threshold;

            int configuration_wait_milliseconds = 2000;

            ::extension::Configuration initialConfig;

            void printState(const Eigen::VectorXd& state);
            Eigen::VectorXd getState(const ::extension::Configuration& walkConfig);
            YAML::Node getWalkConfig(const Eigen::VectorXd& state);
            void saveConfig(const YAML::Node& config);
            void setWalkParameters(const YAML::Node& config);

            FitnessData data;

        public:
            explicit WalkOptimiser(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // namespace optimisation
}  // namespace support
}  // namespace module

#endif  // MODULES_SUPPORT_OPTIMISATION_WALK_OPTIMISER_H
