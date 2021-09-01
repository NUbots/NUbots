#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H

#include <nuclear>
#include <vector>

#include "nsga2/NSGA2.hpp"

#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::support::optimisation::NSGA2EvaluationRequest;

            class NSGA2Optimiser : public NUClear::Reactor {
            private:
                /// @brief Implementation of the NSGA II algorithm, holds the state of the entire optimisation,
                /// including the populations, scores, etc
                nsga2::NSGA2 nsga2Algorithm{};

            public:
                /// @brief Called by the powerplant to build and setup the NSGA2Optimiser reactor.
                explicit NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
