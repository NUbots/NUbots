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

#include "extension/Configuration.h"
#include "message/support/optimisation/DOpE.h"
#include "message/support/optimisation/Episode.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace debug {
    namespace optimisation {

        using extension::Configuration;
        using message::support::optimisation::Episode;
        using message::support::optimisation::Parameters;
        using message::support::optimisation::RegisterOptimisation;
        using message::support::optimisation::RequestParameters;

        TestDOpE::TestDOpE(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), currentParameters() {

            on<Configuration>("TestDOpE.yaml").then([this](const Configuration& /*config*/) {
                // Use configuration here from file TestDOpE.yaml
            });

            on<Trigger<Parameters>>().then([this](const Parameters& params) {

                // If these parameters are for us
                if (params.group == "test_dope") {
                    currentParameters = params;
                }
            });

            // TODO request optimisation parameters from the system
            // Emit an optimisation param request

            on<Every<1, Per<std::chrono::seconds>>>().then([this] {

                auto e = std::make_unique<Episode>();

                e->group      = "test_dope";
                e->generation = currentParameters.generation;
                e->values     = currentParameters.samples.col(0);
                e->covariance = currentParameters.covariance;

                Episode::Fitness fitness;
                fitness.weight = 1.0;

                double f = 0;
                for (int i = 0; i < currentParameters.samples.col(0).size(); ++i) {
                    double v = currentParameters.samples(i, 0) + i;
                    v *= v;
                    f += -v;
                }
                fitness.fitness = f;
                e->fitness.push_back(fitness);

                emit(e);

                // Request a new sample
                auto req      = std::make_unique<RequestParameters>();
                req->group    = "test_dope";
                req->nSamples = 1;
                emit<Scope::DIRECT>(req);
            });

            auto op                           = std::make_unique<RegisterOptimisation>();
            op->group                         = "test_dope";
            op->network                       = true;
            op->parameters.initial.generation = 0;
            op->parameters.initial.estimate   = Eigen::VectorXd::Random(5);
            op->parameters.initial.covariance = Eigen::VectorXd::Constant(5, 0.1).asDiagonal();
            op->parameters.upperBound         = Eigen::VectorXd::Constant(5, std::numeric_limits<double>::max());
            op->parameters.lowerBound         = Eigen::VectorXd::Constant(5, std::numeric_limits<double>::min());
            op->parameters.batchSize          = 10;

            emit<Scope::INITIALIZE>(op);

            auto req      = std::make_unique<RequestParameters>();
            req->group    = "test_dope";
            req->nSamples = 1;
            emit<Scope::INITIALIZE>(req);
        }
    }  // namespace optimisation
}  // namespace debug
}  // namespace module
