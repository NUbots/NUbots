#include "NSGA2Optimiser.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2EvaluationRequest.hpp"
#include "message/support/optimisation/NSGA2FitnessScores.hpp"
#include "message/support/optimisation/NSGA2Terminate.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            using extension::Configuration;

            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;
            using message::support::optimisation::NSGA2Terminate;

            using utility::support::Expression;


            NSGA2Optimiser::NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)) {

                // Read the NSGA2Optimiser.yaml config file and initialize the values we're going to use for the
                // optimisation
                on<Configuration>("NSGA2Optimiser.yaml").then([this](const Configuration& config) {
                    default_gain = config["gains"]["legs"].as<Expression>();

                    // The initial values of the parameters to optimise
                    std::vector<double> initialValues;

                    // Parallel to initialValues, sets the limit (min, max) of each parameter value
                    std::vector<std::pair<double, double>> realLimits;

                    // Extract the initial values and limits and from config file, for all of the parameters
                    auto& walk = config["walk"];
                    for (const auto& element : std::vector<std::string>({std::string("freq"),
                                                                         std::string("double_support_ratio"),
                                                                         std::string("first_step_swing_factor")})) {
                        initialValues.emplace_back(walk[element][0].as<Expression>());
                        realLimits.emplace_back(walk[element][1].as<Expression>(), walk[element][2].as<Expression>());
                    }

                    auto& foot = walk["foot"];
                    for (const auto& element : std::vector<std::string>({std::string("distance"),
                                                                         std::string("rise"),
                                                                         std::string("z_pause"),
                                                                         std::string("apex_phase")})) {
                        initialValues.emplace_back(foot[element][0].as<Expression>());
                        realLimits.emplace_back(foot[element][1].as<Expression>(), foot[element][2].as<Expression>());
                    }
                    auto& put_down = foot["foot"];
                    for (const auto& element : std::vector<std::string>(
                             {std::string("z_offset"), std::string("phase"), std::string("roll_offset")})) {
                        initialValues.emplace_back(put_down[element][0].as<Expression>());
                        realLimits.emplace_back(put_down[element][1].as<Expression>(),
                                                put_down[element][2].as<Expression>());
                    }

                    auto& overshoot = foot["overshoot"];
                    for (const auto& element : std::vector<std::string>({std::string("ratio"), std::string("phase")})) {
                        initialValues.emplace_back(overshoot[element][0].as<Expression>());
                        realLimits.emplace_back(overshoot[element][1].as<Expression>(),
                                                overshoot[element][2].as<Expression>());
                    }

                    auto& trunk = walk["trunk"];
                    for (const auto& element : std::vector<std::string>({std::string("height"),
                                                                         std::string("pitch"),
                                                                         std::string("phase"),
                                                                         std::string("x_offset"),
                                                                         std::string("y_offset"),
                                                                         std::string("swing"),
                                                                         std::string("pause")})) {
                        initialValues.emplace_back(trunk[element][0].as<Expression>());
                        realLimits.emplace_back(trunk[element][1].as<Expression>(), trunk[element][2].as<Expression>());
                    }

                    auto& x_offset = trunk["x_offset_p_coef"];
                    for (const auto& element :
                         std::vector<std::string>({std::string("forward"), std::string("turn")})) {
                        initialValues.emplace_back(x_offset[element][0].as<Expression>());
                        realLimits.emplace_back(x_offset[element][1].as<Expression>(),
                                                x_offset[element][2].as<Expression>());
                    }

                    auto& pitch = trunk["pitch_p_coef"];
                    for (const auto& element :
                         std::vector<std::string>({std::string("forward"), std::string("turn")})) {
                        initialValues.emplace_back(pitch[element][0].as<Expression>());
                        realLimits.emplace_back(pitch[element][1].as<Expression>(), pitch[element][2].as<Expression>());
                    }

                    auto& pause = walk["pause"];
                    for (const auto& element : std::vector<std::string>({std::string("duration")})) {
                        initialValues.emplace_back(trunk[element][0].as<Expression>());
                        realLimits.emplace_back(trunk[element][1].as<Expression>(), trunk[element][2].as<Expression>());
                    }

                    auto& max_step = config["max_step"];
                    for (const auto& element : std::vector<std::string>(
                             {std::string("x"), std::string("y"), std::string("z"), std::string("xy")})) {
                        initialValues.emplace_back(max_step[element][0].as<Expression>());
                        realLimits.emplace_back(max_step[element][1].as<Expression>(),
                                                max_step[element][2].as<Expression>());
                    }

                    // Set up the NSGA2 algorithm based on our config values
                    nsga2Algorithm.SetRealVariableCount(initialValues.size());
                    nsga2Algorithm.SetObjectiveCount(config["num_objectives"].as<int>());
                    nsga2Algorithm.SetContraintCount(config["num_constraints"].as<int>());
                    nsga2Algorithm.SetPopulationSize(config["population_size"].as<int>());
                    nsga2Algorithm.SetTargetGenerations(config["num_generations"].as<int>());
                    nsga2Algorithm.SetRealCrossoverProbability(
                        config["probabilities"]["real"]["crossover"].as<double>());
                    nsga2Algorithm.SetRealMutationProbability(config["probabilities"]["real"]["mutation"].as<double>());
                    nsga2Algorithm.SetEtaC(config["eta"]["C"].as<double>());
                    nsga2Algorithm.SetEtaM(config["eta"]["M"].as<double>());
                    nsga2Algorithm.SetRealVarLimits(realLimits);
                    nsga2Algorithm.SetRandomInitialize(false);
                    nsga2Algorithm.SetInitialRealVars(initialValues);
                    nsga2Algorithm.SetSeed(config["seed"].as<int>());
                });

                on<Startup>().then([this]() {
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

                                // Send the terminate message to end the tests
                                std::unique_ptr<NSGA2Terminate> terminate = std::make_unique<NSGA2Terminate>();
                                terminate->terminateMe                    = 1;
                                emit(terminate);
                            }
                            else if (scores.generation < nsga2Algorithm.generations) {
                                // End the generation and save its data
                                nsga2Algorithm.PostEvaluationAdvance();

                                // Start the next generation (creates the new children for evaluation)
                                nsga2Algorithm.PreEvaluationAdvance();

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
                log<NUClear::INFO>("Evaluating generation", nsga2Algorithm.parentPop->generation, "individual", _id);

                // Create a message to request an evaluation of an individual
                std::unique_ptr<NSGA2EvaluationRequest> request = std::make_unique<NSGA2EvaluationRequest>();
                request->id                                     = _id;
                request->generation                             = _generation;

                // Add the individual's parameters to the message
                request->parameters.freq                          = _reals[0];
                request->parameters.double_support_ratio          = _reals[1];
                request->parameters.first_step_swing_factor       = _reals[2];
                request->parameters.foot.distance                 = _reals[3];
                request->parameters.foot.rise                     = _reals[4];
                request->parameters.foot.z_pause                  = _reals[5];
                request->parameters.foot.apex_phase               = _reals[6];
                request->parameters.foot.put_down.z_offset        = _reals[7];
                request->parameters.foot.put_down.phase           = _reals[8];
                request->parameters.foot.put_down.roll_offset     = _reals[9];
                request->parameters.foot.overshoot.ratio          = _reals[10];
                request->parameters.foot.overshoot.phase          = _reals[11];
                request->parameters.trunk.height                  = _reals[12];
                request->parameters.trunk.pitch                   = _reals[13];
                request->parameters.trunk.phase                   = _reals[14];
                request->parameters.trunk.x_offset                = _reals[15];
                request->parameters.trunk.y_offset                = _reals[16];
                request->parameters.trunk.swing                   = _reals[17];
                request->parameters.trunk.pause                   = _reals[18];
                request->parameters.trunk.x_offset_p_coef.forward = _reals[19];
                request->parameters.trunk.x_offset_p_coef.turn    = _reals[20];
                request->parameters.trunk.pitch_p_coef.forward    = _reals[21];
                request->parameters.trunk.pitch_p_coef.turn       = _reals[22];
                request->parameters.pause.duration                = _reals[23];
                request->parameters.max_step.x                    = _reals[24];
                request->parameters.max_step.y                    = _reals[25];
                request->parameters.max_step.z                    = _reals[26];
                request->parameters.max_step.xy                   = _reals[27];

                // Disable IMU and kick in walk engine
                request->parameters.imu.active  = false;
                request->parameters.kick.length = 0.0;
                request->parameters.kick.phase  = 0.0;
                request->parameters.kick.vel    = 0.0;

                // Set default gains
                request->parameters.gains.legs = default_gain;

                emit(request);
            }
        }  // namespace optimisation
    }      // namespace support
}  // namespace module
