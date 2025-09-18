/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "RotationOptimiser.hpp"

#include <nuclear>

#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {
    using message::support::optimisation::NSGA2EvaluationRequest;
    using utility::support::Expression;

    void RotationOptimiser::setup_nsga2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2_algorithm) {
        NUClear::log<NUClear::LogLevel::INFO>("Rotation Optimiser Setting up NSGA2");

        // Extract the initial values and limits and from config file, for all of the parameters
        auto walk = config["walk"];
        add_parameters(walk);

        auto arms = config["arms"];
        add_parameters(arms);

        auto walk_command = config["walk_command"];
        for (const auto& element : std::vector<std::string>({std::string("rotation")})) {
            param_initial_values.emplace_back(walk_command[element][0].as<Expression>());
            param_limits.emplace_back(walk_command[element][1].as<Expression>(),
                                      walk_command[element][2].as<Expression>());
        }

        walk_config_path     = config["task_config_path"].as<std::string>();
        trial_duration_limit = config["trial_duration_limit"].as<int>();

        // Set configuration for real variables
        NUClear::log<NUClear::LogLevel::INFO>("Real Var Count: ", param_initial_values.size());
        nsga2_algorithm.set_real_variable_count(param_initial_values.size());
        nsga2_algorithm.set_real_var_limits(param_limits);
        nsga2_algorithm.set_initial_real_vars(param_initial_values);

        // Set configuration for binary variables
        nsga2_algorithm.set_bin_variable_count(0);
    }

    void RotationOptimiser::add_parameters(YAML::Node param) {
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

    std::unique_ptr<NSGA2EvaluationRequest> RotationOptimiser::make_evaluation_request(const int id,
                                                                                       const int generation,
                                                                                       std::vector<double> reals) {
        auto request              = std::make_unique<NSGA2EvaluationRequest>();
        request->id               = id;
        request->generation       = generation;
        request->task             = message::support::optimisation::Task::ROTATION;
        request->task_config_path = walk_config_path;

        request->trial_duration_limit = trial_duration_limit;

        // Add the individual's parameters to the message
        request->parameters.real_params = reals;
        return request;
    }

}  // namespace module::support::optimisation
