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

#include "TestDOpE.h"

#include "messages/support/Configuration.h"
#include "messages/support/optimisation/DOpE.h"
#include "messages/support/optimisation/Episode.pb.h"
#include "utility/support/proto_armadillo.h"

namespace modules {
namespace debug {
namespace optimisation {

    using messages::support::Configuration;
    using messages::support::optimisation::Episode;
    using messages::support::optimisation::Parameters;
    using messages::support::optimisation::RequestParameters;
    using messages::support::optimisation::RegisterOptimisation;

    TestDOpE::TestDOpE(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("TestDOpE.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file TestDOpE.yaml
        });

        on<Trigger<Parameters>>().then([this] (const Parameters& params) {
            // If these parameters are for us
            if(params.group == "test_dope") {
                currentParameters = params;
            }
        });

        // TODO request optimisation parameters from the system
        // Emit an optimisation param request

        on<Every<1, std::chrono::seconds>>().then([this] {
            auto e = std::make_unique<Episode>();

            e->set_group("test_dope");
            e->set_generation(currentParameters.generation);
            *e->mutable_values() << currentParameters.samples.col(0);
            *e->mutable_covariance() << currentParameters.covariance;

            emit(e);

            // Request a new sample
            auto req = std::make_unique<RequestParameters>();
            req->group = "test_dope";
            emit<Scope::INITIALIZE>(req);
        });

        auto op = std::make_unique<RegisterOptimisation>();
        op->group = "test_dope";
        op->network = true;
        op->parameters.initial.generation = -1;
        op->parameters.initial.estimate = { 1, 2, 3, 4, 5 };
        op->parameters.initial.covariance = arma::diagmat(arma::vec({ 1, 2, 3, 4, 5 }));
        op->parameters.upperBound = { 10, 10, 10 };
        op->parameters.lowerBound = { 0, 0, 0 };
        op->parameters.batchSize = 10;

        emit<Scope::INITIALIZE>(op);

        auto req = std::make_unique<RequestParameters>();
        req->group = "test_dope";
        req->nSamples =  1;
        emit<Scope::INITIALIZE>(req);
    }
}
}
}
