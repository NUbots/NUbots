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

#include "messages/support/Configuration.h"
#include "messages/support/optimisation/DOpE.h"
#include "messages/support/optimisation/proto/Episode.pb.h"

namespace modules {
namespace support {
namespace optimisation {

    using NUClear::message::NetworkJoin;
    using NUClear::message::NetworkLeave;
    using utility::math::optimisation::Optimiser;
    using utility::math::optimisation::PGAOptimiser;
    using messages::support::Configuration;
    using messages::support::optimisation::proto::Episode;
    using messages::support::optimisation::RequestParameters;
    using messages::support::optimisation::RegisterOptimisation;

    // struct Optimisation {
    //     struct Trial {
    //         arma::vec value;
    //         double fitness
    //     };

    //     bool network;
    //     std::string name;
    //     Trial best;
    //     std::vector<Trial> trials;
    //     int batchSize;
    // };


    DOpE::DOpE(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("DOpE.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file DOpE.yaml
            log("TODO load the configuration from any in progress optimisations here");

            // TODO load any optimisations that are currently in the config file (saved/in progress)
        });

        on<Trigger<NetworkJoin>>().then([this] (const NetworkJoin& joiner) {
            log(joiner.name, "joined");

        //     auto currentState = std::make_unique<BestEpisode>();

        //     // currentState->generation = ;
        //     // currentState->best = ;

        //     // make a message containing the current best episode
        //     // and a list of current episodes
        //     // current episode is not needed

        //     // Emit the current state reliably to the newbie
        //     emit<Scope::NETWORK>(currentState, joiner.name, true);
        });

        // on<Network<BestEpisode>>().then([this] (const BestEpisode& episode) {

        //     // If this optimisation is operating on the network
        //     // If the new best estimates generation is higher,
        //     // or we use a cryptographic hash to pick one randomly (but consistently across the network)
        //     if (episode.generation() > optimisers[episode.name()].bestEpisode.generation()
        //         || cryptographicHash(episode) > cryptographicHash(optimisers[episode.name()].bestEpisode)) {
        //         // TODO we use this new best estimate as our estimate

        //         // TODO go through our optimisers episodes
        //         // If they are not valid for this new optimiser remove them from the list
        //     }
        // });

        on<Network<Episode>>().then([this] (const Episode& episode) {
        //     // Check if this episode is valid for our current best estimate (optimiser is network, epoch is correct, value is in range)

        //     // If so add it to our list of episodes

        //     // If we have reached our batch limit, update our estimate
        //     // and emit our new best estimate result over the network
        });

        on<Trigger<Episode>>().then([this] (const Episode& episode) {
        //     // Check if this episode is valid for our current best estimate

        //     // If so add it to our list of episodes
        //     //      If we have reached our batch limit, update our best estimate and emit the new best estimate over the network
        //     //      Else emit this episode over the network
        });

        on<Trigger<RegisterOptimisation>>().then([this] (const RegisterOptimisation& optimisation) {
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

        //     register {
        //         std::string group;  // The group that this is a part of (a string identifier)
        //         arma::vec values;   // The values that we are currently optimising with
        //         arma::vec weights;  // The starting weights for this optimisation
        //         bool network;       // If we should use the network for this optimisation
        //         bool save;
        //     }
        });

        on<Trigger<RequestParameters>>().then([this] (const RequestParameters& request) {

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
