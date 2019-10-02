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

#include "WalkOptimiser.h"


#include "message/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/optimisation/PGAoptimiser.h"
#include "utility/support/yaml_armadillo.h"

namespace module {
namespace support {
    namespace optimisation {

        using message::behaviour::CancelFixedWalk;
        using message::behaviour::FixedWalkCommand;
        using message::behaviour::FixedWalkFinished;
        using message::behaviour::WalkConfigSaved;
        using message::behaviour::WalkOptimiserCommand;

        using message::input::Sensors;
        using message::input::ServoID;

        using message::motion::ExecuteGetup;
        using message::motion::KillGetup;

        using extension::Configuration;
        using message::support::SaveConfiguration;

        WalkOptimiser::WalkOptimiser(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), initialConfig("Jerry", YAML::Node()) {

            on<Configuration>("WalkOptimiser.yaml").then([this](const Configuration& config) {
                log("Starting up walk optimiser");

                number_of_samples = config["number_of_samples"].as<int>();
                parameter_sigmas.resize(config["parameters_and_sigmas"].size());
                parameter_names.resize(config["parameters_and_sigmas"].size());
                int i = 0;

                for (const auto& parameter : config["parameters_and_sigmas"]) {
                    parameter_names[i]  = parameter.first.as<std::string>();
                    parameter_sigmas[i] = parameter.second.as<double>();
                    i++;
                }

                walk_command.segments.clear();
                for (auto& segment : config["segments"]) {

                    walk_command.segments.push_back(FixedWalkCommand::WalkSegment());
                    walk_command.segments.back().direction   = segment["direction"].as<arma::vec>();
                    walk_command.segments.back().curvePeriod = segment["curvePeriod"].as<double>();

                    walk_command.segments.back().normalisedVelocity = segment["normalisedVelocity"].as<double>();
                    walk_command.segments.back().normalisedAngularVelocity =
                        segment["normalisedAngularVelocity"].as<double>();
                    walk_command.segments.back().duration =
                        std::chrono::milliseconds(int(std::milli::den * segment["duration"].as<double>()));
                }

                getup_cancel_trial_threshold = config["getup_cancel_trial_threshold"].as<uint>();

                configuration_wait_milliseconds = config["configuration_wait_milliseconds"].as<int>();

                emit(std::make_unique<OptimiseWalkCommand>());
            });

            on<Trigger<OptimiseWalkCommand>, Configuration, Sync<WalkOptimiser>>("WalkEngine.yaml")
                .then("Optimise Walk", [this](const OptimiseWalkCommand&, const Configuration& walkConfig) {
                    // Start optimisation
                    std::cerr << "Optimiser command" << std::endl;
                    // Get samples
                    samples = utility::math::optimisation::PGA::getSamples(
                        getState(walkConfig), parameter_sigmas, number_of_samples);
                    // Initialise fitnesses
                    fitnesses.zeros(number_of_samples);
                    // Save the config which we loaded from file
                    initialConfig = walkConfig;
                    // Set the sample we are currently on
                    // Use iteritive evaluation so that more samples can be added at any time
                    currentSample = 0;

                    std::cerr << "Sample: " << currentSample << std::endl;
                    // Apply the parameters to the walk engine
                    setWalkParameters(getWalkConfig(samples.row(currentSample).t()));
                    // Now wait for WalkConfigSaved
                });

            on<Trigger<WalkConfigSaved>, Sync<WalkOptimiser>>([this] {
                std::this_thread::sleep_for(std::chrono::milliseconds(configuration_wait_milliseconds));
                // Start a walk routine
                auto command = std::make_unique<FixedWalkCommand>(walk_command);
                emit(std::move(command));
            });


            on<Every<25, Per<std::chrono::seconds>>, With<Sensors>, Sync<WalkOptimiser>>().then(
                "Walk Data Manager", [this](const Sensors& sensors) {
                    // Record data
                    data.update(sensors);
                });

            on<Trigger<ExecuteGetup>>().then("Getup Recording", [this] {
                // Record the robot falling over
                data.recordGetup();
            });

            on<Trigger<KillGetup>>().then("Getup Recording", [this] {
                data.getupFinished();
                // //If this set of parameters is very bad, stop the trial and send cancel fixed walk command
                if (data.numberOfGetups >= getup_cancel_trial_threshold) {
                    emit(std::make_unique<CancelFixedWalk>());
                }
            });

            on < Trigger<FixedWalkFinished>,
                <Sync<WalkOptimiser>>().then(
                    "Walk Routine Finised",
                    [this] {
                        // Get and reset data
                        fitnesses[currentSample] = data.popFitness();
                        std::cerr << "Sample Done! Fitness: " << fitnesses[currentSample] << std::endl;
                        if (currentSample >= samples.n_rows - 1) {
                            emit(std::make_unique<OptimisationComplete>());
                        }
                        else {
                            // Setup new parameters
                            std::cerr << "Sample:" << ++currentSample << std::endl;
                            setWalkParameters(getWalkConfig(samples.row(currentSample).t()));
                            // Now wait for WalkConfigSaved
                        }
                    });

            on<Trigger<OptimisationComplete>, Sync<WalkOptimiser>>().then("Record Results", [this] {
                // Combine samples
                arma::vec result = utility::math::optimisation::PGA::updateEstimate(samples, fitnesses);

                std::cerr << "Final Result:" << std::endl;
                auto cfg = getWalkConfig(result);
                saveConfig(cfg);
            });
        }

        arma::vec WalkOptimiser::getState(const Configuration& walkConfig) {
            arma::vec state(parameter_names.size());
            std::cerr << "walkConfig.size() = " << walkConfig.config.size() << "\nLoading state:" << std::endl;
            int i = 0;
            for (const std::string& name : parameter_names) {
                state[i++] = walkConfig.config[name].as<double>();
            }
            std::cerr << "Loaded Walk Config State:" << std::endl;
            printState(state);
            return state;
        }

        void WalkOptimiser::printState(const arma::vec& state) {
            std::cerr << "[";
            for (uint i = 0; i < parameter_names.size(); ++i) {
                std::cerr << parameter_names[i] << ": " << state[i] << ", ";
            }
            std::cerr << std::endl;
        }

        YAML::Node WalkOptimiser::getWalkConfig(const arma::vec& state) {
            YAML::Node config(initialConfig.config);
            for (uint i = 0; i < state.size(); ++i) {
                config[parameter_names[i]] = state[i];
            }
            printState(state);
            return config;
        }

        void WalkOptimiser::saveConfig(const YAML::Node& config) {
            auto saveConfig    = std::make_unique<SaveConfiguration>();
            saveConfig->path   = WalkOptimiserCommand::CONFIGURATION_PATH;
            saveConfig->config = config.as<std::string>();
            emit(std::move(saveConfig));
        }

        void WalkOptimiser::setWalkParameters(const YAML::Node& config) {
            auto command        = std::make_unique<WalkOptimiserCommand>();
            command->walkConfig = config;
            emit(std::move(command));
        }

        double FitnessData::popFitness() {
            std::cerr << "Calculating fitness: " << std::endl;
            double stabilityFitness = (M_PI_4 - tilt.mean()) / M_PI_4;
            std::cerr << "stabilityFitness = " << stabilityFitness << std::endl;
            double getupFitness = (numberOfGetups == 0 ? 1 : 1 / double(1 + numberOfGetups));
            std::cerr << "numberOfGetups = " << numberOfGetups << std::endl;
            std::cerr << "getupFitness = " << getupFitness << std::endl;
            numberOfGetups = 0;
            tilt.reset();
            // Reset all data
            return getupFitness + stabilityFitness;
        }
        void FitnessData::update(const message::input::Sensors& sensors) {
            if (recording) {
                arma::vec3 verticalKinematics  = sensors.camToGround.submat(0, 2, 2, 2);
                arma::vec3 verticalOrientation = sensors.kinematicsCamToGround.submat(0, 2, 2, 2);
                double tiltMag = utility::math::angle::acos_clamped(arma::dot(verticalOrientation, verticalKinematics));
                if (std::fabs(tiltMag) < M_PI_4) {
                    tilt(tiltMag);
                }
            }
        }
        void FitnessData::recordGetup() {
            numberOfGetups++;
            recording = false;
        }
        void FitnessData::getupFinished() {
            recording = true;
        }
    }  // namespace optimisation
}  // namespace support
}  // namespace module
