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

#include "messages/input/Sensors.h"
#include "messages/motion/WalkCommand.h"    
#include "messages/behaviour/FixedWalkCommand.h"
#include "messages/motion/GetupCommand.h"
#include "messages/support/Configuration.h"
#include "utility/support/armayamlconversions.h"
 


namespace modules {
    namespace support {
        namespace optimisation {

            using messages::support::Configuration;
            using messages::motion::WalkCommand;
            using messages::behaviour::FixedWalkCommand;
            using messages::behaviour::FixedWalkFinished;
            using messages::input::Sensors;
            using messages::motion::ExecuteGetup;
            using messages::motion::KillGetup;


            WalkOptimiser::WalkOptimiser(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)) {

                on<Trigger<Configuration<WalkOptimiser>>>([this](const Configuration<WalkOptimiser>& config){
                    // std::vector<string> parameterNames;
                    // parameterNames = config["PARAMETERS_TO_OPTIMISE"];
                    auto command = std::make_unique<FixedWalkCommand>();
                    for(auto& segment : config["segments"]){
                        command->segments.push_back(FixedWalkCommand::WalkSegment());
                        command->segments.back().direction = segment["direction"].as<arma::vec>();
                        NUClear::log("direction",command->segments.back().direction);
                        command->segments.back().curvePeriod = segment["curvePeriod"].as<double>();
                        NUClear::log("curvePeriod",command->segments.back().curvePeriod);

                        command->segments.back().normalisedVelocity = segment["normalisedVelocity"].as<double>();                
                        NUClear::log("normalisedVelocity",command->segments.back().normalisedVelocity);
                        command->segments.back().normalisedAngularVelocity = segment["normalisedAngularVelocity"].as<double>();                
                        NUClear::log("normalisedAngularVelocity",command->segments.back().normalisedAngularVelocity);
                        command->segments.back().duration = std::chrono::milliseconds(int(std::milli::den * segment["duration"].as<double>()));
                        NUClear::log("duration",std::chrono::duration_cast<std::chrono::seconds>(command->segments.back().duration).count());
                    }
                    emit(std::move(command));
                });

                on< Trigger< Every<100, Per<std::chrono::seconds>> >, With<Sensors> >("Walk Data Manager", [this](const time_t& t, const Sensors& sensors){
                    // data.update(sensors);
                });

                on<Trigger<ExecuteGetup>>("Getup Recording", [this](const ExecuteGetup& command){
                    // data.update(getup);
                    NUClear::log("!!! Getup Detected by Optimiser !!!");
                });

                // on<Trigger<OptimiseWalkCommand>>([this]("Optimise Walk", const OptimiseWalkCommand& command){
                //     // member arma::mat samples = PGA::getSamples(command.state_vec, 0.01 * command);
                    
                //     // sampleNumber = 0;
                //     // emit(std::make_unique<WalkParameters>(samples[sampleNumber]));
                //     // emit(std::make_unique<WaypointWalk>(start_time));
                //     // emit(std::make_unique<CircularWalk>(start_time));
                    
                // });

                on<Trigger<FixedWalkFinished>> ("Optimise Walk", [this](const FixedWalkFinished& command){
                    
                    NUClear::log("!!! FixedWalkFinished !!!");
                    // fitnesses.push_back(data.calculateFitness());
                    // if(sampleNumber == samples.n_rows-1){
                    //     emit(std::make_unique<OptimisationComplete>);
                    // }
                    // emit(std::make_unique<WalkParameters>(samples[++sampleNumber]));
                    // emit(std::make_unique<WaypointWalk>(start_time));
                    // emit(std::make_unique<CircularWalk>(start_time));
                });
                


                // on<Trigger<OptimisationComplete>([this]("Record Results Script", const OptimisationComplete& results)){
                //     // PGA::updateEstimate(samples, results.fitnesses);
                // });
            }   



        } //optimisation
    } // support
} // modules
