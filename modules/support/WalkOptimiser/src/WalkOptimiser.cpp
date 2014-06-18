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



namespace modules {
    namespace support {
        namespace optimisation {


            WalkOptimiser::WalkOptimiser(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)) {

                on<Trigger<Configuration<WalkOptimiser>>>([this](const Configuration<WalkOptimiser>& config)){
                    std::vector<string> parameterNames;
                    parameterNames = config["PARAMETERS_TO_OPTIMISE"];
                });

                on< Trigger< Every<100, Per<std::chrono::seconds>> >, With<Sensors> >([this]("Walk Data Manager", const time_t& t, const Sensors& sensors){
                    if(t > current_script.end_time()){
                        current_script = scripts.pop_front();
                        fitnesses.push_back(current_fitness);
                    }
                    emit(current_script.getWalkCommand(t));

                    current_fitness.update(sensors);

                    if(scripts.empty()){
                        emit(OptimisationComplete(current_fitness));
                    }
                });

                on<Trigger<GetUp>>[this]("Getup Recording", const GetUp& sensors){
                    current_fitness.update(getup);
                });

                on<Trigger<OptimiseWalkCommand>>([this]("Optimise Walk", const OptimiseWalkCommand& command){
                    member arma::mat samples = PGA::getSamples(command.state_vec, 0.01 * command);
                    for(sample : samples){
                        scripts.push_back(WalkScript(sample, start_time));
                    }
                });

                on<Trigger<OptimisationComplete>([this]("Record Results Script", const OptimisationComplete& results)){
                    PGA::updateEstimate(samples, results.fitnesses);
                });
            }   



        } //optimisation
    } // support
} // modules
