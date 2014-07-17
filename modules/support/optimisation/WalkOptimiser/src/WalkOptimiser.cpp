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
#include "messages/input/ServoID.h"

namespace modules {
    namespace support {
        namespace optimisation {

            using messages::behaviour::FixedWalkCommand;
            using messages::behaviour::FixedWalkFinished;
            using messages::behaviour::CancelFixedWalk;
            using messages::behaviour::WalkOptimiserCommand;
            using messages::behaviour::WalkConfigSaved;

            using messages::input::Sensors;
            using messages::input::ServoID;

            using messages::motion::ExecuteGetup;
            using messages::motion::KillGetup;

            using messages::support::SaveConfiguration;
            using messages::support::Configuration;


            WalkOptimiser::WalkOptimiser(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)),
                initialConfig("Jerry", YAML::Node()) {

                on<Trigger<Configuration<WalkOptimiser>>>([this](const Configuration<WalkOptimiser>& config){

                    std::cerr << "Starting up walk optimiser" << std::endl;

                    number_of_samples = config["number_of_samples"].as<int>();
                    parameter_sigmas.resize( config["parameters_and_sigmas"].size());
                    parameter_names.resize( config["parameters_and_sigmas"].size());
                    int i = 0;

                    for(const auto& parameter : config["parameters_and_sigmas"]) {
                        parameter_names[i] = parameter.first.as<std::string>();
                        parameter_sigmas[i] = parameter.second.as<double>();
                        i++;
                    }

                    walk_command.segments.clear();
                    for(auto& segment : config["segments"]){

                        walk_command.segments.push_back(FixedWalkCommand::WalkSegment());
                        walk_command.segments.back().direction = segment["direction"].as<arma::vec>();
                        walk_command.segments.back().curvePeriod = segment["curvePeriod"].as<double>();

                        walk_command.segments.back().normalisedVelocity = segment["normalisedVelocity"].as<double>();
                        walk_command.segments.back().normalisedAngularVelocity = segment["normalisedAngularVelocity"].as<double>();
                        walk_command.segments.back().duration = std::chrono::milliseconds(int(std::milli::den * segment["duration"].as<double>()));
                    }

                    getup_cancel_trial_threshold = config["getup_cancel_trial_threshold"].as<uint>();

                    configuration_wait_milliseconds = config["configuration_wait_milliseconds"].as<int>();

                    emit(std::make_unique<OptimiseWalkCommand>());
                });

                on<Trigger<OptimiseWalkCommand>, With<Configuration<WalkOptimiserCommand>>, Options<Sync<WalkOptimiser>>>("Optimise Walk", [this](const OptimiseWalkCommand&, const Configuration<WalkOptimiserCommand>& walkConfig){

                    //Start optimisation
                    std::cerr << "Optimiser command" << std::endl;
                    //Get samples
                    samples = utility::math::optimisation::PGA::getSamples(getState(walkConfig), parameter_sigmas, number_of_samples);
                    //Initialise fitnesses
                    fitnesses.zeros(number_of_samples);
                    //Save the config which we loaded from file
                    initialConfig = walkConfig;
                    //Set the sample we are currently on
                    //Use iteritive evaluation so that more samples can be added at any time
                    currentSample = 0;

                    std::cerr << "Sample: " << currentSample <<std::endl;
                    //Apply the parameters to the walk engine
                    setWalkParameters(getWalkConfig(samples.row(currentSample).t()));
                    //Now wait for WalkConfigSaved

                });

                on<Trigger<WalkConfigSaved>, Options<Sync<WalkOptimiser>>>([this](const WalkConfigSaved&){
                    std::this_thread::sleep_for(std::chrono::milliseconds(configuration_wait_milliseconds));
                    //Start a walk routine
                    auto command = std::make_unique<FixedWalkCommand>(walk_command);
                    emit(std::move(command));
                });


                on< Trigger< Every<25, Per<std::chrono::seconds>> >, With<Sensors>, Options<Sync<WalkOptimiser>> >("Walk Data Manager", [this](const time_t&, const Sensors& sensors){
                    //Record data
                    data.update(sensors);
                });

                on<Trigger<ExecuteGetup>>("Getup Recording", [this](const ExecuteGetup&){
                    //Record the robot falling over
                    data.recordGetup();

                });

                on<Trigger<KillGetup>>("Getup Recording", [this](const KillGetup&){
                    data.getupFinished();
                    // //If this set of parameters is very bad, stop the trial and send cancel fixed walk command
                    if(data.numberOfGetups >= getup_cancel_trial_threshold){
                        emit(std::make_unique<CancelFixedWalk>());
                    }
                });

                on<Trigger<FixedWalkFinished>, Options<Sync<WalkOptimiser>> > ("Walk Routine Finised", [this](const FixedWalkFinished&){
                    //Get and reset data
                    fitnesses[currentSample] = data.popFitness();
                    std::cerr << "Sample Done! Fitness: " << fitnesses[currentSample] << std::endl;
                    if(currentSample >= samples.n_rows-1){
                        emit(std::make_unique<OptimisationComplete>());
                    } else {
                        //Setup new parameters
                        std::cerr << "Sample:" << ++currentSample <<std::endl;
                        setWalkParameters(getWalkConfig(samples.row(currentSample).t()));
                        //Now wait for WalkConfigSaved
                    }
                });

                on<Trigger<OptimisationComplete>, Options<Sync<WalkOptimiser>>>("Record Results", [this]( const OptimisationComplete&){
                    //Combine samples
                    arma::vec result = utility::math::optimisation::PGA::updateEstimate(samples, fitnesses);

                    std::cerr << "Final Result:" <<std::endl;
                    auto cfg = getWalkConfig(result);
                    saveConfig(cfg);
                });

                //TODO network
                // on<Trigger<Network<WalkOptimiserCommandParameters>>>("Add Sample",[this](const Configuration<WalkOptimiserCommand>& walkConfig)){
                //     //samples.push_back(getState()));
                    // fitnesses.resize(number_of_samples);
                //
                // });


            }

            arma::vec WalkOptimiser::getState(const Configuration<WalkOptimiserCommand>& walkConfig){
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

            YAML::Node WalkOptimiser::getWalkConfig(const arma::vec& state){
                YAML::Node config(initialConfig.config);
                for(uint i = 0; i < state.size(); ++i){
                    config[parameter_names[i]] = state[i];
                }
                printState(state);
                return config;
            }

            void WalkOptimiser::saveConfig(const YAML::Node& config){
                auto saveConfig = std::make_unique<SaveConfiguration>();
                saveConfig->path = WalkOptimiserCommand::CONFIGURATION_PATH;
                saveConfig->config = config;
                emit(std::move(saveConfig));
            }

            void WalkOptimiser::setWalkParameters(const YAML::Node& config){
                auto command = std::make_unique<WalkOptimiserCommand>();
                command->walkConfig = config;
                emit(std::move(command));
            }

            double FitnessData::popFitness(){
                std::cerr << "Calculating fitness: " <<std::endl;
                double stabilityFitness = (M_PI_4 - tilt.mean()) / M_PI_4;
                std::cerr << "stabilityFitness = " << stabilityFitness <<std::endl;
                double getupFitness = (numberOfGetups == 0 ? 1 : 1 / double(1+numberOfGetups));
                std::cerr << "numberOfGetups = " <<  numberOfGetups <<std::endl;
                std::cerr << "getupFitness = " <<  getupFitness <<std::endl;
                numberOfGetups = 0;
                tilt.reset();
                //Reset all data
                return getupFitness + stabilityFitness;
            }
            void FitnessData::update(const messages::input::Sensors& sensors){
                if(recording){
                    arma::vec3 verticalKinematics = sensors.orientationCamToGround.submat(0,2,2,2);
                    arma::vec3 verticalOrientation = sensors.kinematicsCamToGround.submat(0,2,2,2);
                    double tiltMag = std::acos(arma::dot(verticalOrientation, verticalKinematics));
                    if(std::fabs(tiltMag) < M_PI_4){
                        tilt(tiltMag);
                    }
                }
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
