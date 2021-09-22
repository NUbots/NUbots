#include <nuclear>

#include "StandOptimiser.hpp"

#include "nsga2/NSGA2.hpp"
#include "extension/Configuration.hpp"
#include "utility/support/yaml_expression.hpp"

#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using utility::support::Expression;
            using message::support::optimisation::NSGA2EvaluationRequest;

            void StandOptimiser::SetupNSGA2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2Algorithm) {
                NUClear::log<NUClear::INFO>("Stand Optimiser Setting up NSGA2");
                // The initial values of the parameters to optimise
                std::vector<double> paramInitialValues;

                // Parallel to paramInitialValues, sets the limit (min, max) of each parameter value
                std::vector<std::pair<double, double>> paramLimits;

                // Extract the initial values and limits and from config file, for all of the parameters
                scriptPath = config["script_path"].as<std::string>();

                auto& stand = config["stand"];
                for (const auto& element : stand) {
                    //This is iterating through each frame of the script
                    paramInitialValues.emplace_back(element["duration"][0].as<Expression>());
                    paramLimits.emplace_back(element["duration"][1].as<Expression>(), element["duration"][2].as<Expression>());
                }

                // Set configuration for real variables
                NUClear::log<NUClear::INFO>("Real Var Count: ", paramInitialValues.size());
                nsga2Algorithm.SetRealVariableCount(paramInitialValues.size());
                nsga2Algorithm.SetRealVarLimits(paramLimits);
                nsga2Algorithm.SetInitialRealVars(paramInitialValues);

                // Set configuration for binary variables
                nsga2Algorithm.SetBinVariableCount(0);
            }

            std::unique_ptr<NSGA2EvaluationRequest> StandOptimiser::MakeEvaluationRequest(const int id, const int generation, std::vector<double> reals) {
                auto request = std::make_unique<NSGA2EvaluationRequest>();
                request->id = id;
                request->generation = generation;
                request->task = "stand";
                // Add the individual's parameters to the message
                request->parameters.real_params = reals;
                return request;
            }

        }  // namespace optimisation
    }      // namespace support
}  // namespace module
