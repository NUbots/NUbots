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

#include "DOpE.h"

#include <armadillo>
#include <format.h>

#include "messages/support/Configuration.h"
#include "messages/support/optimisation/DOpE.h"
#include "messages/support/optimisation/Episode.pb.h"
#include "messages/support/optimisation/Estimate.pb.h"

namespace modules {
namespace support {
namespace optimisation {

    using NUClear::message::NetworkJoin;
    using NUClear::message::NetworkLeave;
    using utility::math::optimisation::Optimiser;
    using utility::math::optimisation::PGAOptimiser;
    using messages::support::Configuration;
    using messages::support::optimisation::Episode;
    using messages::support::optimisation::Estimate;
    using messages::support::optimisation::RequestParameters;
    using messages::support::optimisation::RegisterOptimisation;

    DOpE::DOpE(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("DOpE.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file DOpE.yaml
            log("TODO load the configuration from any in progress optimisations here");

            // TODO load any optimisations that are currently in the config file (saved/in progress)
        });

        on<Trigger<NetworkJoin>>().then("Distrubute Initial Optimisation", [this] (const NetworkJoin& joiner) {

            log<NUClear::INFO>(fmt::format("{} ({}) joined the optimisation network", joiner.name, joiner.udpPort));

            for (auto& op : optimisations) {
                // If this is a network optimisation
                if(op.second.network) {
                    // Get the current state for this
                    auto current = op.second.optimiser->estimate();
                    auto e = std::make_unique<Estimate>();

                    // Set our group
                    e->set_group(op.first);

                    // Add our generation
                    e->set_generation(current.generation);

                    // Add our values
                    for (uint i = 0; i < current.estimate.n_elem; ++i) {
                        e->mutable_values()->add_v(current.estimate[i]);
                    }

                    // Add our covariance
                    for (uint y = 0; y < current.covariance.n_cols; ++y) {
                        auto row = e->mutable_covariance()->add_v();
                        for (uint x = 0; x < current.covariance.n_rows; ++x) {
                            row->add_v(current.covariance(x, y));
                        }
                    }

                    // Add our episodes
                    for (auto& episode : op.second.episodes) {
                        *e->add_episode() = episode;
                    }

                    // Send it to the remote
                    emit<Scope::NETWORK>(e, joiner.name, true);
                }
            }
        });

        on<Network<Estimate>>().then("Network Estimate", [this] (const NetworkSource& src, const Estimate& estimate) {

            log<NUClear::INFO>(fmt::format("Estimate {} gen {} received from {}", estimate.group(), estimate.generation(), src.name));

            auto el = optimisations.find(estimate.group());
            if (el != optimisations.end()) {
                auto& opt = el->second;

                if(opt.optimiser->estimate().generation < estimate.generation()) {
                    // TODO This is a new estimate, use this
                    // TODO also save this best estimate in the config
                    // TODO also check if we have any episodes that are valid and not in this episodes data
                }
                else if (opt.optimiser->estimate().generation == estimate.generation()) {
                    // TODO This is the same as our existing generation
                    // TODO Find some arbritrary way to work out which is better
                }
            }
            else {
                // TODO should we add this? or ignore it?
            }
        });

        on<Network<Episode>>().then("Network Episode", [this] (const NetworkSource& src, const Episode& episode) {

            log<NUClear::INFO>(fmt::format("Episode for {} gen {} received from {}", episode.group(), episode.generation(), src.name));

            // If we have this optimisation
            auto el = optimisations.find(episode.group());
            if (el != optimisations.end()) {
                // Check if this episode is relevant for us

                // Check if this episode is valid for our current best estimate (optimiser is network, epoch is correct, value is in range)

                // If so add it to our list of episodes
                // Also save our optimisation state again

                // If we have reached our batch limit, update our estimate
                // and emit our new best estimate result over the network
            }
        });

        on<Trigger<Episode>>().then("Local Episode", [this] (const Episode& episode) {

            // If we have this optimisation
            auto el = optimisations.find(episode.group());
            if (el != optimisations.end()) {
                //     // Check if this episode is valid for our current best estimate

                //     // If so add it to our list of episodes
                //     //      If we have reached our batch limit, update our best estimate and emit the new best estimate over the network
                //     //      Else emit this episode over the network
            }
            else {
                // If we don't have an optimiser for this, this is an error
                log<NUClear::ERROR>(fmt::format("Episode for {} gen {} generated for unregistered optimisation", episode.group(), episode.generation()));
            }
        });

        on<Trigger<RegisterOptimisation>>().then("Register Optimisation", [this] (const RegisterOptimisation& optimisation) {
            // Add this optimisation to the list
            auto item = optimisations.find(optimisation.group);
            if (item == optimisations.end()) {
                // Add this new optimisation
                log("Adding a new optimisation for", optimisation.group);

                optimisations[optimisation.group] = Optimisation {
                    optimisation.network,
                    std::make_unique<PGAOptimiser>(optimisation.params),
                    std::vector<Episode>()
                };
            }
            else {
                // Check if the size of the vectors are the same
                // If they are not, log an error here saying that another
                // Optimisation with the same name was already registered
                log<NUClear::ERROR>("The optimisation,", optimisation.group, "was already registered as a different type");
            }
        });

        on<Trigger<RequestParameters>>().then("Request Optimisation Parameters", [this] (const RequestParameters& request) {

        //     // Generate a list of paramters for this request type
        //     // Send them out

        //     params {
        //         std::string group;
        //         int generation;
        //         arma::vec values;
        //     }
        });
    }
}
}
}
