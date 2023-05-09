#include "MultiPathOptimiser.hpp"

#include <fstream>  // ifstream
#include <nuclear>
#include <sstream>  //istringstream

#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {
    using message::support::optimisation::NSGA2EvaluationRequest;
    using utility::support::Expression;

    void MultiPathOptimiser::SetupNSGA2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2_algorithm) {
        NUClear::log<NUClear::INFO>("Multipath Optimiser Setting up NSGA2");
        // The initial values of the parameters to optimise
        std::vector<double> param_initial_values;

        // Parallel to param_initial_values, sets the limit (min, max) of each parameter value
        std::vector<std::pair<double, double>> param_limits;

        std::vector<std::vector<double>> population_data;


        std::string input_file_name =
            "../NUbots/module/support/optimisation/NSGA2Optimiser/data/config/Supplied_Pop_Reals.csv";

        population_data = readInPopulationFile(input_file_name);
        NUClear::log<NUClear::INFO>("Data file size is", population_data.size());


        // bool supp_pop =

        // Extract the initial values and limits and from config file, for all of the parameters
        auto walk = config["walk"];
        for (const auto& element :
             std::vector<std::string>({std::string("freq"), std::string("double_support_ratio")})) {
            param_initial_values.emplace_back(walk[element][0].as<Expression>());
            param_limits.emplace_back(walk[element][1].as<Expression>(), walk[element][2].as<Expression>());
        }

        auto foot = walk["foot"];
        for (const auto& element : std::vector<std::string>({std::string("distance"), std::string("rise")})) {
            param_initial_values.emplace_back(foot[element][0].as<Expression>());
            param_limits.emplace_back(foot[element][1].as<Expression>(), foot[element][2].as<Expression>());
        }

        auto trunk = walk["trunk"];
        for (const auto& element : std::vector<std::string>({std::string("height"),
                                                             std::string("pitch"),
                                                             std::string("x_offset"),
                                                             std::string("y_offset"),
                                                             std::string("swing"),
                                                             std::string("pause")})) {
            param_initial_values.emplace_back(trunk[element][0].as<Expression>());
            param_limits.emplace_back(trunk[element][1].as<Expression>(), trunk[element][2].as<Expression>());
        }

        auto pause = walk["pause"];
        for (const auto& element : std::vector<std::string>({std::string("duration")})) {
            param_initial_values.emplace_back(pause[element][0].as<Expression>());
            param_limits.emplace_back(pause[element][1].as<Expression>(), pause[element][2].as<Expression>());
        }

        auto walk_command = config["walk_command"];
        for (const auto& element : std::vector<std::string>({std::string("velocity")})) {
            param_initial_values.emplace_back(walk_command[element][0].as<Expression>());
            param_limits.emplace_back(walk_command[element][1].as<Expression>(),
                                      walk_command[element][2].as<Expression>());
        }

        for (const auto& element : std::vector<std::string>({std::string("rotation")})) {
            param_initial_values.emplace_back(walk_command[element][0].as<Expression>());
            param_limits.emplace_back(walk_command[element][1].as<Expression>(),
                                      walk_command[element][2].as<Expression>());
        }

        quintic_walk_path    = config["task_config_path"].as<std::string>();
        trial_duration_limit = config["trial_duration_limit"].as<int>();
        // supplied

        // Set configuration for real variables
        NUClear::log<NUClear::INFO>("Real Var Count: ", param_initial_values.size());
        nsga2_algorithm.SetRealVariableCount(param_initial_values.size());
        nsga2_algorithm.SetRealVarLimits(param_limits);
        nsga2_algorithm.SetInitialRealVars(param_initial_values);
        nsga2_algorithm.SetInitialPopulationRealVars(population_data);
        nsga2_algorithm.setSuppliedPop(true);

        // Set configuration for binary variables
        nsga2_algorithm.SetBinVariableCount(0);
    }

    std::unique_ptr<NSGA2EvaluationRequest> MultiPathOptimiser::MakeEvaluationRequest(const int id,
                                                                                      const int generation,
                                                                                      std::vector<double> reals) {
        auto request              = std::make_unique<NSGA2EvaluationRequest>();
        request->id               = id;
        request->generation       = generation;
        request->task             = "multipath";
        request->task_config_path = quintic_walk_path;

        request->trial_duration_limit = trial_duration_limit;

        // Add the individual's parameters to the message
        request->parameters.real_params = reals;
        return request;
    }

    std::vector<std::vector<double>> MultiPathOptimiser::readInPopulationFile(const std::string file_name) {
        std::vector<std::vector<double>> data;
        std::ifstream inputFile(file_name);
        int l = 0;

        while (inputFile) {
            l++;
            std::string s;
            if (!getline(inputFile, s))
                break;
            if (s[0] != '#') {
                std::istringstream ss(s);
                std::vector<double> record;

                while (ss) {
                    std::string line;
                    if (!getline(ss, line, ','))
                        break;
                    try {
                        record.push_back(stof(line));
                    }
                    catch (const std::invalid_argument& e) {
                        NUClear::log<NUClear::INFO>("NaN found in file ");
                        e.what();
                    }
                }

                data.push_back(record);
            }
        }

        if (!inputFile.eof()) {
            // cerr << "Could not read file " << inputFileName << "\n";
            NUClear::log<NUClear::INFO>("Could not read file", file_name);
            // std::__throw_invalid_argument("File not found.");
        }

        return data;
    }

}  // namespace module::support::optimisation
