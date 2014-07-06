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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_SUPPORT_OPTIMISATION_WALK_OPTIMISER_H
#define MODULES_SUPPORT_OPTIMISATION_WALK_OPTIMISER_H

#include <nuclear>
#include <armadillo>

#include "messages/input/Sensors.h"
#include "messages/motion/GetupCommand.h"
#include "messages/support/Configuration.h"
#include "messages/behaviour/FixedWalkCommand.h"


namespace modules {
    namespace support {
        namespace optimisation {

            struct OptimiseWalkCommand{};
            struct OptimisationComplete{};

            struct WalkEngineConfig{
                static constexpr const char* CONFIGURATION_PATH = "WalkEngine.yaml";
            };

            class FitnessData {
            public:
                uint numberOfGetups = 0;
                bool recording;
                double popFitness();
                void update(const messages::input::Sensors& sensors);
                void recordGetup();
                void getupFinished();
            };

            class WalkOptimiser : public NUClear::Reactor {
            private:
                messages::behaviour::FixedWalkCommand walk_command;
                std::vector<std::string> parameter_names;
                arma::vec parameter_sigmas;
                arma::vec fitnesses;

                uint currentSample;
                arma::mat samples;
                uint number_of_samples;

                uint getup_cancel_trial_threshold;

                messages::support::Configuration<WalkEngineConfig> initialConfig;

                static constexpr const char* backupLocation = "WalkEngine_Optimised.yaml";

                void printState(const arma::vec& state);
                arma::vec getState(const messages::support::Configuration<WalkEngineConfig>& walkConfig);
                messages::support::Configuration<WalkEngineConfig> getWalkConfig(const arma::vec& state);
                void saveConfig(const messages::support::Configuration<WalkEngineConfig>& config);
                void saveGoodConfigBackup(const messages::support::Configuration<WalkEngineConfig>& config);

                FitnessData data;
            public:
                static constexpr const char* CONFIGURATION_PATH = "WalkOptimiser.yaml";
                explicit WalkOptimiser(std::unique_ptr<NUClear::Environment> environment);
            };

        } //optimisation
    }  // support
}  // modules

#endif  // MODULES_SUPPORT_NUBUGGER_H

