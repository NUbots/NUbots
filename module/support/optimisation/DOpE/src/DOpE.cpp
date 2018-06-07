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

#include <fmt/format.h>
#include <google/protobuf/util/message_differencer.h>
#include <armadillo>

#include "extension/Configuration.h"
#include "message/support/optimisation/DOpE.h"
#include "message/support/optimisation/Episode.h"
#include "message/support/optimisation/Estimate.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace support {
    namespace optimisation {

        using google::protobuf::util::MessageDifferencer;

        using extension::Configuration;
        using message::support::optimisation::Episode;
        using message::support::optimisation::Estimate;
        using message::support::optimisation::Parameters;
        using message::support::optimisation::RegisterOptimisation;
        using message::support::optimisation::RequestParameters;
        using NUClear::message::NetworkJoin;
        using NUClear::message::NetworkLeave;
        using utility::math::optimisation::Optimiser;
        using utility::math::optimisation::OptimiserEstimate;
        using utility::math::optimisation::PGAOptimiser;

        void DOpE::sendEstimateUpdate(const Optimisation& opt, const std::string& target) {

            // Get the current state for this
            const auto& current = opt.optimiser->estimate();
            auto e              = std::make_unique<Estimate>();

            // Set our group
            e->group = opt.group;

            // Add our generation
            e->generation = current.generation;

            // Add our values and covariance
            e->values     = current.estimate;
            e->covariance = current.covariance;

            log<NUClear::FATAL>("Current Estimate", convert<double>(current.estimate).t());

            // Add our episodes
            for (auto& episode : opt.episodes) {
                e->episode.push_back(episode);
            }
            for (auto& episode : opt.estimateEpisodes) {
                e->estimate_episode.push_back(episode);
            }

            // Send it to the remote
            emit<Scope::NETWORK>(e, target, true);
        }

        void DOpE::processBatch(Optimisation& opt) {

            // We are updating our batch
            log<NUClear::INFO>(
                fmt::format("Processing {}/{} episode batch for {}", opt.episodes.size(), opt.batchSize, opt.group));

            arma::vec fitnesses(opt.episodes.size());
            arma::mat samples(opt.episodes.size(), opt.optimiser->estimate().estimate.rows());

            for (uint i = 0; i < opt.episodes.size(); ++i) {

                // Make our combined fitness
                fitnesses[i] = 0;
                for (auto& f : opt.episodes[i].fitness) {
                    fitnesses[i] += f.fitness * f.weight;
                }

                for (uint j = 0; j < samples.n_cols; ++j) {
                    samples(i, j) = opt.episodes[i].values[j];
                }
            }

            // Update our optimiser
            opt.optimiser->updateEstimate(samples, fitnesses);

            // Clear our episodes list
            opt.episodes.clear();

            // Emit our new best estimate over the network
            if (opt.network) {
                sendEstimateUpdate(opt);
            }
        }

        void DOpE::saveOptimisationState(const Optimisation& /*opt*/) {
            // TODO save the optimisation
            // TODO merge the current yaml file with our new yaml file
            // TODO save the yaml file
            // TODO this will trigger the configuration which we don't wanna do
        }

        DOpE::DOpE(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), optimisations() {

            on<Configuration, Sync<DOpE>>("DOpE.yaml").then([this](const Configuration& /*config*/) {
                // Use configuration here from file DOpE.yaml
                log("TODO load the configuration from any in progress optimisations here");

                // TODO load any optimisations that are currently in the config file (saved/in progress)
            });

            on<Trigger<NetworkJoin>, Sync<DOpE>>().then(
                "Distrubute Initial Optimisation", [this](const NetworkJoin& joiner) {
                    log<NUClear::INFO>(fmt::format("{} joined the optimisation network", joiner.name));

                    for (auto& op : optimisations) {
                        // If this is a network optimisation
                        if (op.second.network) {
                            // Send it to our joiner
                            sendEstimateUpdate(op.second, joiner.name);
                        }
                    }
                });

            on<Network<Estimate>, Sync<DOpE>>().then(
                "Network Estimate", [this](const NetworkSource& src, const Estimate& estimate) {
                    log<NUClear::INFO>(fmt::format(
                        "Estimate {} gen {} received from {}", estimate.group, estimate.generation, src.name));

                    auto el = optimisations.find(estimate.group);
                    if (el != optimisations.end()) {
                        auto& opt = el->second;

                        // If we are doing a network optimisation
                        if (opt.network) {

                            // Get our current estimate
                            const auto& currentEstimate = opt.optimiser->estimate();

                            // We use the network version if it has a newer generation
                            // or we are the same generation, we choose based on the bitwise xor of the estimate
                            if (currentEstimate.generation < estimate.generation) {
                                // || (opt.optimiser->estimate().generation == estimate.generation() && fitness <
                                // fitness) {

                                OptimiserEstimate e;
                                e.generation = estimate.generation;
                                e.estimate   = estimate.values;
                                e.covariance = estimate.covariance;

                                // Reset with these parameters
                                opt.optimiser->reset(e);

                                std::vector<Episode> newEpisodes(estimate.episode.begin(), estimate.episode.end());
                                std::vector<Episode> newEstimateEpisodes(estimate.estimate_episode.begin(),
                                                                         estimate.estimate_episode.end());

                                // Go through our episodes and throw out ones that are in either list
                                // or are no longer valid for this state
                                for (auto it = opt.episodes.begin(); it != opt.episodes.end();) {

                                    // Work out if this episode is in either list
                                    auto f    = [&it](const Episode& e) { return e == *it; };
                                    auto newE = std::find_if(newEpisodes.begin(), newEpisodes.end(), f);
                                    auto newPE =
                                        std::find_if(newEstimateEpisodes.begin(), newEstimateEpisodes.end(), f);

                                    if (newE != newEpisodes.end() || newPE != newEstimateEpisodes.end()) {
                                        it = opt.episodes.erase(it);
                                    }
                                    else {
                                        ++it;
                                    }
                                }

                                // Insert our newEpisodes into our episode list
                                opt.episodes.insert(opt.episodes.begin(), newEpisodes.begin(), newEpisodes.end());

                                // Swap to our new previous episodes
                                opt.estimateEpisodes = newEstimateEpisodes;

                                // Save our optimisation state to the config file
                                saveOptimisationState(opt);
                            }
                            else if (currentEstimate.generation == estimate.generation) {
                                // TODO some random selection of the two

                                // TODO we need to use this ones prevEpisodes
                                // TODO we need to recover samples in our prevEpisodes that are not in this prevEpisodes
                            }
                        }
                    }
                    else {
                        // TODO should we add this? or ignore it?
                    }
                });

            on<Network<Episode>, Sync<DOpE>>().then(
                "Network Episode", [this](const NetworkSource& src, const Episode& episode) {
                    // If we have this optimisation
                    auto el = optimisations.find(episode.group);
                    if (el != optimisations.end()) {
                        auto& opt = el->second;

                        // If this optimiser works on the network
                        if (opt.network) {
                            log<NUClear::DEBUG>(fmt::format("Network episode for {}({}) received from {}",
                                                            episode.group,
                                                            episode.generation,
                                                            src.name));

                            // If we don't already have this episode and it is valid for our optimiser
                            if (  // std::find(opt.episodes.begin(), opt.episodes.end(), episode) == opt.episodes.end()
                                  // std::find(opt.previousEpisodes.begin(), opt.previousEpisodes.end(), episode) ==
                                  // opt.episodes.end()
                                opt.optimiser->validSample(episode)) {

                                // TODO fix the checking if we already have this sample

                                opt.episodes.push_back(episode);
                                if (opt.episodes.size() >= opt.batchSize) {
                                    processBatch(opt);
                                }

                                // Save our optimisation state to the config file
                                saveOptimisationState(opt);
                            }
                        }
                    }
                });

            on<Trigger<Episode>, Sync<DOpE>>().then("Local Episode", [this](const Episode& episode) {
                // If we have this optimisation
                auto el = optimisations.find(episode.group);
                if (el != optimisations.end()) {
                    auto& opt = el->second;

                    log<NUClear::DEBUG>(fmt::format("Local episode for {}({})", episode.group, episode.generation));

                    // If this episode is valid for our optimiser
                    if (opt.optimiser->validSample(episode)) {

                        opt.episodes.push_back(episode);

                        if (opt.episodes.size() >= opt.batchSize) {

                            processBatch(opt);
                        }
                        // If we are networked send out this episode
                        else if (opt.network) {
                            auto e = std::make_unique<Episode>(episode);
                            emit<Scope::NETWORK>(e, "", true);
                        }

                        // Save our optimisation state to the config file
                        saveOptimisationState(opt);
                    }
                }
                else {
                    // If we don't have an optimiser for this, this is an error
                    log<NUClear::ERROR>(fmt::format("Episode for {} gen {} generated for unregistered optimisation",
                                                    episode.group,
                                                    episode.generation));
                }
            });

            on<Trigger<RegisterOptimisation>, Sync<DOpE>>().then(
                "Register Optimisation", [this](const RegisterOptimisation& optimisation) {
                    // Add this optimisation to the list
                    auto item = optimisations.find(optimisation.group);
                    if (item == optimisations.end()) {
                        // Add this new optimisation
                        log<NUClear::INFO>("Adding a new optimisation for", optimisation.group);

                        optimisations[optimisation.group] =
                            Optimisation{optimisation.group,
                                         optimisation.network,
                                         optimisation.parameters.batchSize,
                                         std::make_unique<PGAOptimiser>(optimisation.parameters),
                                         std::vector<Episode>(),
                                         std::vector<Episode>()};
                    }
                    else {
                        // TODO this may already have been loaded via either the config file, or the network

                        // Check if the size of the vectors are the same
                        // If they are not, log an error here saying that another
                        // Optimisation with the same name was already registered
                        log<NUClear::ERROR>(
                            "The optimisation,", optimisation.group, "was already registered as a different type");
                    }
                });

            on<Trigger<RequestParameters>, Sync<DOpE>>().then(
                "Request Optimisation Parameters", [this](const RequestParameters& request) {
                    auto el = optimisations.find(request.group);
                    if (el != optimisations.end()) {
                        auto& opt = el->second;

                        auto p = std::make_unique<Parameters>();

                        p->group      = request.group;
                        p->generation = opt.optimiser->estimate().generation;
                        p->samples    = convert<double>(opt.optimiser->getSamples(request.nSamples));
                        p->covariance = opt.optimiser->estimate().covariance;

                        log<NUClear::DEBUG>(fmt::format(
                            "Generating {} parameters for {}({})", request.nSamples, p->group, p->generation));

                        emit(p);
                    }
                    else {
                        log<NUClear::ERROR>("The optimisation,", request.group, "was requested but does not exist");
                    }
                });
        }
    }  // namespace optimisation
}  // namespace support
}  // namespace module
