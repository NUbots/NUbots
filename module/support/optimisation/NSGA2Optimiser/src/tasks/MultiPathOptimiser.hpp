#ifndef MODULE_SUPPORT_OPTIMISATION_MULTIPATH_HPP
#define MODULE_SUPPORT_OPTIMISATION_MULTIPATH_HPP

#include <nuclear>
#include <vector>

#include "OptimiserTask.hpp"
#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {
    using message::support::optimisation::NSGA2EvaluationRequest;

    class MultiPathOptimiser : public OptimiserTask {
    public:
        void setup_nsga2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2_algorithm);
        std::unique_ptr<NSGA2EvaluationRequest> make_evaluation_request(const int id,
                                                                        const int generation,
                                                                        std::vector<double> reals);
        std::vector<std::vector<double>> read_in_population_file(const std::string flie_name);

    private:
        /// @brief The maximum duration in seconds that a trial can take
        int trial_duration_limit;

        /// @brief Path for the Walk config file
        std::string walk_config_path;

        /// @brief The initial values of the parameters to optimise
        std::vector<double> param_initial_values;

        /// @brief Parallel to param_initial_values, sets the limit (min, max) of each parameter value
        std::vector<std::pair<double, double>> param_limits;

        void add_parameters(YAML::Node param);
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_MULTIPATH_HPP
