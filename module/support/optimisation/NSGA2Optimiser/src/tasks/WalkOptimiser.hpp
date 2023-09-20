#ifndef MODULE_SUPPORT_OPTIMISATION_WALKOPTIMISER_HPP
#define MODULE_SUPPORT_OPTIMISATION_WALKOPTIMISER_HPP

#include <nuclear>
#include <vector>

#include "OptimiserTask.hpp"
#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {
    using message::support::optimisation::NSGA2EvaluationRequest;

    class WalkOptimiser : public OptimiserTask {
    public:
        void setup_nsga2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2_algorithm);
        std::unique_ptr<NSGA2EvaluationRequest> make_evaluation_request(const int id,
                                                                        const int generation,
                                                                        std::vector<double> reals);

    private:
        int trial_duration_limit;
        std::string walk_path;

        /// @brief The initial values of the parameters to optimise
        std::vector<double> param_initial_values;

        /// @brief Parallel to param_initial_values, sets the limit (min, max) of each parameter value
        std::vector<std::pair<double, double>> param_limits;

        void add_parameters(YAML::Node param);
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_WALKOPTIMISER_HPP
