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

#include "WalkOptimiser.h"


#include "utility/support/armayamlconversions.h"
#include "utility/math/optimisation/PGAoptimiser.h"

namespace modules {
    namespace support {
        namespace optimisation {

            using messages::support::Configuration;
            using messages::behaviour::FixedWalkCommand;
            using messages::behaviour::FixedWalkFinished;
            using messages::input::Sensors;
            using messages::motion::ExecuteGetup;
            using messages::motion::KillGetup;
            using messages::support::SaveConfiguration;


            WalkOptimiser::WalkOptimiser(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)),
                initialConfig("Jerry", YAML::Node()) {

                on<Trigger<Configuration<WalkOptimiser>>>([this](const Configuration<WalkOptimiser>& config){

                    std::cerr << "Starting up walk optimiser" << std::endl;

                    number_of_samples = config["number_of_samples"].as<int>();
                    parameter_sigmas.resize( config.config["parameters_and_sigmas"].size());
                    parameter_names.resize( config.config["parameters_and_sigmas"].size());
                    int i = 0;

                    for(const auto& parameter : config["parameters_and_sigmas"]) {
                        parameter_names[i] = parameter.first.as<std::string>();
                        parameter_sigmas[i] = parameter.second.as<double>();
                        i++;
                    }

                    for(auto& segment : config["segments"]){
                        walk_command.segments.push_back(FixedWalkCommand::WalkSegment());
                        walk_command.segments.back().direction = segment["direction"].as<arma::vec>();
                        walk_command.segments.back().curvePeriod = segment["curvePeriod"].as<double>();

                        walk_command.segments.back().normalisedVelocity = segment["normalisedVelocity"].as<double>();
                        walk_command.segments.back().normalisedAngularVelocity = segment["normalisedAngularVelocity"].as<double>();
                        walk_command.segments.back().duration = std::chrono::milliseconds(int(std::milli::den * segment["duration"].as<double>()));
                    }

                    getup_cancel_trial_threshold = config["getup_cancel_trial_threshold"].as<double>();

                    emit(std::make_unique<OptimiseWalkCommand>());
                });

                on<Trigger<OptimiseWalkCommand>, With<Configuration<WalkEngineConfig>> >("Optimise Walk", [this]( const OptimiseWalkCommand&, const Configuration<WalkEngineConfig>& walkConfig){
                    std::cerr << "Optimiser command" << std::endl;
                    samples = utility::math::optimisation::PGA::getSamples(getState(walkConfig), parameter_sigmas, number_of_samples);
                    fitnesses.zeros(number_of_samples);
                    initialConfig = walkConfig;

                    currentSample = 0;

                    std::cerr << "Sample: " << currentSample <<std::endl;
                    saveConfig(getWalkConfig(samples.row(currentSample).t()));

                    auto command = std::make_unique<FixedWalkCommand>(walk_command);
                    emit(std::move(command));
                });

                on< Trigger< Every<25, Per<std::chrono::seconds>> >, With<Sensors> >("Walk Data Manager", [this](const time_t&, const Sensors& sensors){
                    data.update(sensors);
                });

                on<Trigger<ExecuteGetup>>("Getup Recording", [this](const ExecuteGetup&){
                    data.recordGetup();
                    if(data.numberOfGetups > getup_cancel_trial_threshold){
                        emit(std::make_unique<FixedWalkFinished>());
                    }
                });
                on<Trigger<KillGetup>>("Getup Recording", [this](const KillGetup&) {
                    data.getupFinished();
                });

                on<Trigger<FixedWalkFinished>> ("Walk Routine Finised", [this](const FixedWalkFinished&){
                    //Get and reset data
                    fitnesses[currentSample] = data.popFitness();
                    std::cerr << "Sample Done! Fitness: " << fitnesses[currentSample] << std::endl;
                    if(currentSample >= samples.n_rows-1){
                        emit(std::make_unique<OptimisationComplete>());
                    } else {
                        //Setup new parameters
                        std::cerr << "Sample:" << ++currentSample <<std::endl;
                        saveConfig(getWalkConfig(samples.row(currentSample).t()));
                        //Start a walk routine
                        auto command = std::make_unique<FixedWalkCommand>(walk_command);
                        emit(std::move(command));
                    }
                });

                on<Trigger<OptimisationComplete> >("Record Results", [this](const OptimisationComplete&){
                    //Combine samples
                    arma::vec result = utility::math::optimisation::PGA::updateEstimate(samples, fitnesses);

                    std::cerr << "Final Result:"<<std::endl;
                    auto cfg = getWalkConfig(result);
                    saveConfig(cfg);
                    saveGoodConfigBackup(cfg);
                });

                //TODO network
                // on<Trigger<Network<WalkEngineConfigParameters>>>("Add Sample",[this](const Configuration<WalkEngineConfig>& walkConfig)){
                //     //samples.push_back(getState()));
                    // fitnesses.resize(number_of_samples);
                //
                // });


            }

            arma::vec WalkOptimiser::getState(const Configuration<WalkEngineConfig>& walkConfig){
                arma::vec state(parameter_names.size());
                std::cerr << "walkConfig.size() = " << walkConfig.config.size() << "\nLoading state:"<< std::endl;
                int i = 0;
                for(const std::string& name : parameter_names){
                    state[i++] = walkConfig.config[name].as<double>();
                }
                std::cerr << "Loaded Walk Config State:"<<std::endl;
                printState(state);
                return state;
            }
            void WalkOptimiser::printState(const arma::vec& state){
                std::cerr << "[";
                for(uint i = 0; i < parameter_names.size(); ++i){
                    std::cerr << parameter_names[i] << ": " << state[i] <<", ";
                }
                std::cerr << std::endl;
            }

            Configuration<WalkEngineConfig> WalkOptimiser::getWalkConfig(const arma::vec& state){
                Configuration<WalkEngineConfig> config(initialConfig);
                for(uint i = 0; i < state.size(); ++i){
                    config.config[parameter_names[i]] = state[i];
                }
                printState(state);
                return config;
            }

            void WalkOptimiser::saveConfig(const Configuration<WalkEngineConfig>& config){
                auto saveConfig = std::make_unique<SaveConfiguration>();
                saveConfig->path = WalkEngineConfig::CONFIGURATION_PATH;
                saveConfig->config = config.config;
                emit(std::move(saveConfig));
            }

            void WalkOptimiser::saveGoodConfigBackup(const Configuration<WalkEngineConfig>& config){
                auto saveConfig = std::make_unique<SaveConfiguration>();
                saveConfig->path = backupLocation;
                saveConfig->config = config.config;
                emit(std::move(saveConfig));
            }

            double FitnessData::popFitness(){
                double result = (numberOfGetups == 0 ? 1 : 1 / double(1+numberOfGetups));
                numberOfGetups = 0;
                //Reset all data
                return result;
            }
            void FitnessData::update(const messages::input::Sensors& sensors){
                (void)sensors;
            }
            void FitnessData::recordGetup(){
                numberOfGetups++;
                recording = false;
            }
            void FitnessData::getupFinished(){
                recording = true;
            }
        } //optimisation
    } // support
} // modules
