#include "WalkOptimiser.hpp"

#include <nuclear>

#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {
    using message::support::optimisation::NSGA2EvaluationRequest;
    using utility::support::Expression;

    void WalkOptimiser::setup_nsga2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2_algorithm) {
        NUClear::log<NUClear::INFO>("Walk Optimiser Setting up NSGA2");

        // Extract the initial values and limits and from config file, for all of the parameters
        auto walk = config["walk"];
        add_parameters(walk);

        auto arms = config["arms"];
        add_parameters(arms);

        auto walk_command = config["walk_command"];
        for (const auto& element : std::vector<std::string>({std::string("velocity")})) {
            param_initial_values.emplace_back(walk_command[element][0].as<Expression>());
            param_limits.emplace_back(walk_command[element][1].as<Expression>(),
                                      walk_command[element][2].as<Expression>());
        }

        walk_config_path            = config["task_config_path"].as<std::string>();
        trial_duration_limit = config["trial_duration_limit"].as<int>();

        // Set configuration for real variables
        NUClear::log<NUClear::INFO>("Real Var Count: ", param_initial_values.size());
        nsga2_algorithm.set_real_variable_count(param_initial_values.size());
        nsga2_algorithm.set_real_var_limits(param_limits);
        nsga2_algorithm.set_initial_real_vars(param_initial_values);

        // Set configuration for binary variables
        nsga2_algorithm.set_bin_variable_count(0);
    }

    void WalkOptimiser::add_parameters(YAML::Node param) {
        // Load the children parameters of the given node
        for (const auto& element : param) {
            YAML::Node child = element.second;
            if (child.IsMap()) {
                add_parameters(child);
            }
            else {
                param_initial_values.emplace_back(child[0].as<Expression>());
                param_limits.emplace_back(child[1].as<Expression>(), child[2].as<Expression>());
            }
        }
    }

    std::unique_ptr<NSGA2EvaluationRequest> WalkOptimiser::make_evaluation_request(const int id,
                                                                                   const int generation,
                                                                                   std::vector<double> reals) {
        auto request              = std::make_unique<NSGA2EvaluationRequest>();
        request->id               = id;
        request->generation       = generation;
        request->task             = "walk";
        request->task_config_path = walk_config_path;

        request->trial_duration_limit = trial_duration_limit;

        // Add the individual's parameters to the message
        request->parameters.real_params = reals;
        return request;
    }

}  // namespace module::support::optimisation
