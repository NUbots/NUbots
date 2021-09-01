#ifndef MODULE_SUPPORT_OPTIMISATION_OPTIMISERTASK_H
#define MODULE_SUPPORT_OPTIMISATION_OPTIMISERTASK_H

#include <nuclear>
#include <vector>

#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::support::optimisation::NSGA2EvaluationRequest;

            /// TODO: Joel Wong 2021-09-01: I designed this as an "interface", but I can't work out how to make the methods virtual and still have everything compile
            class OptimiserTask {
            public:
                virtual void SetupNSGA2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2Algorithm) = 0;
                virtual std::unique_ptr<NSGA2EvaluationRequest> MakeEvaluationRequest(const int id, const int generation, std::vector<double> reals) = 0;
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_OPTIMISERTASK_H
