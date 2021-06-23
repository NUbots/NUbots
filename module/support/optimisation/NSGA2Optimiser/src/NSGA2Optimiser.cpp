#include "NSGA2Optimiser.hpp"

#include "extension/Configuration.hpp"

#include "message/platform/webots/WebotsReady.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2EvaluationRequest.hpp"
#include "message/support/optimisation/NSGA2FitnessScores.hpp"
#include "message/support/optimisation/NSGA2Terminate.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            using extension::Configuration;

            using message::platform::webots::OptimisationCommand;
            using message::platform::webots::WebotsReady;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;
            using message::support::optimisation::NSGA2Terminate;

            using utility::support::Expression;


            NSGA2Optimiser::NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)) {

                // Read the NSGA2Optimiser.yaml config file and initialize the values we're going to use for the
                // optimisation
                on<Configuration>("NSGA2Optimiser.yaml").then([this](const Configuration& config) {
                    default_leg_gains = config["gains"]["legs"].as<Expression>();

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

                    auto& pause = walk["pause"];  // config['walk']['pause']
                    for (const auto& element : std::vector<std::string>({std::string("duration")})) {
                        paramInitialValues.emplace_back(pause[element][0].as<Expression>());
                        paramLimits.emplace_back(pause[element][1].as<Expression>(),
                                                 pause[element][2].as<Expression>());
                    }

                    // Set up the NSGA2 algorithm based on our config values
                    nsga2Algorithm.SetRealVariableCount(paramInitialValues.size());
                    nsga2Algorithm.SetObjectiveCount(config["num_objectives"].as<int>());
                    nsga2Algorithm.SetContraintCount(config["num_constraints"].as<int>());
                    nsga2Algorithm.SetPopulationSize(config["population_size"].as<int>());
                    nsga2Algorithm.SetTargetGenerations(config["num_generations"].as<int>());
                    nsga2Algorithm.SetRealCrossoverProbability(
                        config["probabilities"]["real"]["crossover"].as<double>());
                    nsga2Algorithm.SetRealMutationProbability(config["probabilities"]["real"]["mutation"].as<double>());
                    nsga2Algorithm.SetEtaC(config["eta"]["C"].as<double>());
                    nsga2Algorithm.SetEtaM(config["eta"]["M"].as<double>());
                    nsga2Algorithm.SetRealVarLimits(paramLimits);
                    nsga2Algorithm.SetRandomInitialize(false);
                    nsga2Algorithm.SetInitialRealVars(paramInitialValues);

                    // Zero out binary things
                    nsga2Algorithm.SetBinCrossoverProbability(0.0);
                    nsga2Algorithm.SetBinMutationProbability(0.0);

                    // This seg faults, SetSeed in the library dereferences a null pointer
                    nsga2Algorithm.SetSeed(config["seed"].as<int>());

                    // Not sure what this should be, but it has to be set
                    nsga2Algorithm.SetBinVariableCount(0);
                });

                on<Startup>().then([this]() {
                    // Create a message to request an evaluation of an individual
                    std::unique_ptr<WebotsReady> message = std::make_unique<WebotsReady>();
                    message->sim_time                    = 0;

                    log<NUClear::INFO>("starting up in 4 seconds");
                    emit<Scope::DELAY>(message, std::chrono::seconds(4));
                });

                on<Trigger<WebotsReady>, Single>().then([this](const WebotsReady& message) {
                    log<NUClear::INFO>("webots ready, starting first evaluation");

                    // On system startup, initialize the algorithm
                    bool initSucceeded = nsga2Algorithm.PreEvaluationInitialize() == 0;

                    // If initialisation succeeded, evaluate the first individual of the first generation
                    // Subsequent individuals will be evaluated after we get the evaluation scores for this individual
                    // (from the NSGA2FitnessScores trigger)
                    if (initSucceeded) {
                        requestIndEvaluation(0,
                                             nsga2Algorithm.parentPop->generation,
                                             nsga2Algorithm.parentPop->GetIndReals(0));
                    }
                });

                on<Trigger<NSGA2FitnessScores>, Single>().then([this](const NSGA2FitnessScores& scores) {
                    log<NUClear::INFO>("got evaluation fitness scores");

                    // An individual has been evaluation and we've got the scores. This updates the
                    // algorithm with the score, and evaluates the next individual.

                    if (scores.generation == 1)  // INITIAL GENERATION
                    {
                        // Tell the algorithm the evaluation scores for this individual
                        nsga2Algorithm.parentPop->SetIndObjectiveScore(scores.id, scores.objScore);
                        nsga2Algorithm.parentPop->SetIndConstraints(scores.id, scores.constraints);

                        // If we have more individuals then evaluate next individual
                        if (scores.id < nsga2Algorithm.popSize - 1) {
                            requestIndEvaluation(scores.id + 1,
                                                 nsga2Algorithm.parentPop->generation,
                                                 nsga2Algorithm.parentPop->GetIndReals(scores.id + 1));
                        }
                        // If this was the last individual, end the current generation and start the next generation
                        else if (scores.id == nsga2Algorithm.popSize - 1) {
                            // End the first generation
                            nsga2Algorithm.PostEvaluationInitialize();

                            // Start the next generation (creates the new children for evaluation)
                            nsga2Algorithm.PreEvaluationAdvance();

                            log("advanced to generation", nsga2Algorithm.childPop->generation);

                            // Evaluate the first individual in the new generation
                            requestIndEvaluation(0,
                                                 nsga2Algorithm.childPop->generation,
                                                 nsga2Algorithm.childPop->GetIndReals(0));
                        }
                        // Otherwise we have an individual that's out of bounds
                        // TODO: perhaps make the above `else if` an `else`
                        else {
                            log<NUClear::INFO>("error: individual number out of bounds");
                        }
                    }
                    else  // FOLLOWING GENERATIONS
                    {
                        // Tell the algorithm the evaluation scores for this individual
                        nsga2Algorithm.childPop->SetIndObjectiveScore(scores.id, scores.objScore);
                        nsga2Algorithm.childPop->SetIndConstraints(scores.id, scores.constraints);

                        // If we have more individuals then evaluate next individual
                        if (scores.id < nsga2Algorithm.popSize - 1) {
                            requestIndEvaluation(scores.id + 1,
                                                 nsga2Algorithm.childPop->generation,
                                                 nsga2Algorithm.childPop->GetIndReals(scores.id + 1));
                        }
                        // If this was the last individual, end the current generation and start the next generation
                        else if (scores.id == nsga2Algorithm.popSize - 1) {
                            // If this was the final generation, finish up
                            if (scores.generation == nsga2Algorithm.generations) {
                                // End the generation and save its data
                                nsga2Algorithm.PostEvaluationAdvance();

                                // Report the population of our final generation
                                nsga2Algorithm.ReportFinalGenerationPop();

                                log<NUClear::INFO>("NSGA2 evaluation finished!");

                                // Tell the evaluator to finish up
                                std::unique_ptr<NSGA2Terminate> terminate = std::make_unique<NSGA2Terminate>();
                                terminate->terminateMe                    = 1;
                                emit(terminate);

                                // Tell Webots to terminate
                                std::unique_ptr<OptimisationCommand> msg = std::make_unique<OptimisationCommand>();
                                msg->command                             = OptimisationCommand::CommandType::TERMINATE;
                                emit(msg);
                            }
                            else if (scores.generation < nsga2Algorithm.generations) {
                                // End the generation and save its data
                                nsga2Algorithm.PostEvaluationAdvance();

                                // Start the next generation (creates the new children for evaluation)
                                nsga2Algorithm.PreEvaluationAdvance();

                                log("advanced to generation", nsga2Algorithm.childPop->generation);

                                // Evaluate the first individual in the new generation
                                requestIndEvaluation(0,
                                                     nsga2Algorithm.childPop->generation,
                                                     nsga2Algorithm.childPop->GetIndReals(0));
                            }
                        }
                        // Otherwise we have an individual that's out of bounds
                        // TODO: perhaps make the above `else if` an `else`
                        else {
                            log<NUClear::INFO>("error: individual number out of bounds");
                        }
                    }
                });
            }

            void NSGA2Optimiser::requestIndEvaluation(int _id, int _generation, const std::vector<double>& _reals) {
                log<NUClear::INFO>("Evaluating generation", _generation, "individual", _id);

                // Create a message to request an evaluation of an individual
                std::unique_ptr<NSGA2EvaluationRequest> request = std::make_unique<NSGA2EvaluationRequest>();
                request->id                                     = _id;
                request->generation                             = _generation;

                // Add the individual's parameters to the message
                request->parameters.freq                 = _reals[0];
                request->parameters.double_support_ratio = _reals[1];
                request->parameters.foot.distance        = _reals[2];
                request->parameters.foot.rise            = _reals[3];
                request->parameters.trunk.height         = _reals[4];
                request->parameters.trunk.pitch          = _reals[5];
                request->parameters.trunk.x_offset       = _reals[6];
                request->parameters.trunk.y_offset       = _reals[7];
                request->parameters.trunk.swing          = _reals[8];
                request->parameters.trunk.pause          = _reals[9];
                request->parameters.pause.duration       = _reals[10];

                // Set default leg gains
                request->parameters.gains.legs = default_leg_gains;

                emit(request);
            }
        }  // namespace optimisation
    }      // namespace support
}  // namespace module
