#include <nuclear>

#include "WalkOptimiser.hpp"

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

            void WalkOptimiser::SetupNSGA2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2Algorithm) {
                                NUClear::log<NUClear::INFO>("Walk Optimiser Setting up NSGA2");
                // The initial values of the parameters to optimise
                std::vector<double> paramInitialValues;

                // Parallel to paramInitialValues, sets the limit (min, max) of each parameter value
                std::vector<std::pair<double, double>> paramLimits;

                // Extract the initial values and limits and from config file, for all of the parameters
                auto& walk = config["walk"];
                for (const auto& element :
                        std::vector<std::string>({std::string("freq"), std::string("double_support_ratio")})) {
                    paramInitialValues.emplace_back(walk[element][0].as<Expression>());
                    paramLimits.emplace_back(walk[element][1].as<Expression>(), walk[element][2].as<Expression>());
                }

                auto& foot = walk["foot"];
                for (const auto& element :
                        std::vector<std::string>({std::string("distance"), std::string("rise")})) {
                    paramInitialValues.emplace_back(foot[element][0].as<Expression>());
                    paramLimits.emplace_back(foot[element][1].as<Expression>(), foot[element][2].as<Expression>());
                }

                auto& trunk = walk["trunk"];
                for (const auto& element : std::vector<std::string>({std::string("height"),
                                                                        std::string("pitch"),
                                                                        std::string("x_offset"),
                                                                        std::string("y_offset"),
                                                                        std::string("swing"),
                                                                        std::string("pause")})) {
                    paramInitialValues.emplace_back(trunk[element][0].as<Expression>());
                    paramLimits.emplace_back(trunk[element][1].as<Expression>(),
                                                trunk[element][2].as<Expression>());
                }

                auto& pause = walk["pause"];
                for (const auto& element : std::vector<std::string>({std::string("duration")})) {
                    paramInitialValues.emplace_back(pause[element][0].as<Expression>());
                    paramLimits.emplace_back(pause[element][1].as<Expression>(),
                                                pause[element][2].as<Expression>());
                }

                auto& walk_command = config["walk_command"];
                for (const auto& element :
                        std::vector<std::string>({std::string("velocity")})) {
                    paramInitialValues.emplace_back(walk_command[element][0].as<Expression>());
                    paramLimits.emplace_back(walk_command[element][1].as<Expression>(), walk_command[element][2].as<Expression>());
                }

                trial_duration_limit = config["trial_duration_limit"].as<int>();

                // Set configuration for real variables
                NUClear::log<NUClear::INFO>("Real Var Count 1: ", paramInitialValues.size());
                nsga2Algorithm.SetRealVariableCount(paramInitialValues.size());
                nsga2Algorithm.SetRealVarLimits(paramLimits);
                nsga2Algorithm.SetInitialRealVars(paramInitialValues);

                // Set configuration for binary variables
                nsga2Algorithm.SetBinVariableCount(0);
            }

            std::unique_ptr<NSGA2EvaluationRequest> WalkOptimiser::MakeEvaluationRequest(const int id, const int generation, std::vector<double> reals) {
                auto request = std::make_unique<NSGA2EvaluationRequest>();
                request->id = id;
                request->generation = generation;
                request->task = "walk";

                request->parameters.trial_duration_limit = trial_duration_limit;

                // Add the individual's parameters to the message
                request->parameters.freq                 = reals[0];
                request->parameters.double_support_ratio = reals[1];
                request->parameters.foot.distance        = reals[2];
                request->parameters.foot.rise            = reals[3];
                request->parameters.trunk.height         = reals[4];
                request->parameters.trunk.pitch          = reals[5];
                request->parameters.trunk.x_offset       = reals[6];
                request->parameters.trunk.y_offset       = reals[7];
                request->parameters.trunk.swing          = reals[8];
                request->parameters.trunk.pause          = reals[9];
                request->parameters.pause.duration       = reals[10];
                request->parameters.velocity             = reals[11];
                return request;
            }

        }  // namespace optimisation
    }      // namespace support
}  // namespace module
