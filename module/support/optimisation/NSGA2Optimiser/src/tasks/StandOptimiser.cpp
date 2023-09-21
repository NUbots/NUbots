#include "StandOptimiser.hpp"

#include <nuclear>

#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {
    using message::support::optimisation::NSGA2EvaluationRequest;
    using utility::support::Expression;

    void StandOptimiser::setup_nsga2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2_algorithm) {
        NUClear::log<NUClear::INFO>("Stand Optimiser Setting up NSGA2");

        // Extract the initial values and limits and from config file, for all of the parameters
        script_path = config["task_config_path"].as<std::string>();

        auto stand = config["stand"];
        for (const auto& element : stand) {
            // This is iterating through each frame of the script
            param_initial_values.emplace_back(element["duration"][0].as<Expression>());
            param_limits.emplace_back(element["duration"][1].as<Expression>(), element["duration"][2].as<Expression>());
        }

        // Set configuration for real variables
        NUClear::log<NUClear::INFO>("Real Var Count: ", param_initial_values.size());
        nsga2_algorithm.set_real_variable_count(param_initial_values.size());
        nsga2_algorithm.set_real_var_limits(param_limits);
        nsga2_algorithm.set_initial_real_vars(param_initial_values);

        // Set configuration for binary variables
        nsga2_algorithm.set_bin_variable_count(0);
    }

    std::unique_ptr<NSGA2EvaluationRequest> StandOptimiser::make_evaluation_request(const int id,
                                                                                    const int generation,
                                                                                    std::vector<double> reals) {
        auto request              = std::make_unique<NSGA2EvaluationRequest>();
        request->id               = id;
        request->generation       = generation;
        request->task             = "stand";
        request->task_config_path = script_path;
        // Add the individual's parameters to the message
        request->parameters.real_params = reals;
        return request;
    }

}  // namespace module::support::optimisation
