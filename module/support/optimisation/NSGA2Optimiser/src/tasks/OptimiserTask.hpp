#ifndef MODULE_SUPPORT_OPTIMISATION_OPTIMISERTASK_HPP
#define MODULE_SUPPORT_OPTIMISATION_OPTIMISERTASK_HPP

#include <nuclear>
#include <vector>

#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {
    using message::support::optimisation::NSGA2EvaluationRequest;

    class OptimiserTask {
    public:
        virtual void setup_nsga2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2_algorithm)  = 0;
        virtual std::unique_ptr<NSGA2EvaluationRequest> make_evaluation_request(const int id,
                                                                                const int generation,
                                                                                std::vector<double> reals) = 0;
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_OPTIMISERTASK_HPP
