#include "NSGA2Optimiser.hpp"

#include "tasks/MultiPathOptimiser.hpp"
#include "tasks/RotationOptimiser.hpp"
#include "tasks/StandOptimiser.hpp"
#include "tasks/StrafeOptimiser.hpp"
#include "tasks/WalkOptimiser.hpp"

#include "extension/Configuration.hpp"

#include "message/platform/webots/WebotsReady.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {

    using extension::Configuration;

    using message::platform::webots::OptimisationCommand;
    using message::platform::webots::WebotsReady;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2EvaluatorReady;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2Terminate;

    using utility::support::Expression;

    NSGA2Optimiser::NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Read NSGA2Optimiser.yaml file and initialize the values we're going to use for the optimisation
        on<Configuration>("NSGA2Optimiser.yaml").then([this](const Configuration& config) {
            log<NUClear::INFO>("Trying to setup NSGA2");

            // Set up the NSGA2 algorithm based on our config values
            nsga2_algorithm.SetObjectiveCount(config["num_objectives"].as<int>());
            nsga2_algorithm.SetContraintCount(config["num_constraints"].as<int>());
            nsga2_algorithm.SetPopulationSize(config["population_size"].as<int>());
            nsga2_algorithm.SetTargetGenerations(config["max_generations"].as<int>());

            nsga2_algorithm.SetRealCrossoverProbability(config["probabilities"]["real"]["crossover"].as<double>());
            nsga2_algorithm.SetBinCrossoverProbability(config["probabilities"]["binary"]["crossover"].as<double>());
            nsga2_algorithm.SetRealMutationProbability(config["probabilities"]["real"]["mutation"].as<double>());
            nsga2_algorithm.SetBinMutationProbability(config["probabilities"]["binary"]["mutation"].as<double>());
            nsga2_algorithm.SetEtaC(config["eta"]["C"].as<double>());
            nsga2_algorithm.SetEtaM(config["eta"]["M"].as<double>());
            nsga2_algorithm.SetSeed(config["seed"].as<int>());

            auto task_type = config["task"].as<std::string>();

            if (task_type == "walk") {
                log<NUClear::INFO>("Task type is Walk");
                task = std::make_unique<WalkOptimiser>();
            }
            else if (task_type == "strafe") {
                log<NUClear::INFO>("Task type is Strafe");
                task = std::make_unique<StrafeOptimiser>();
            }
            else if (task_type == "rotation") {
                log<NUClear::INFO>("Task type is Rotate");
                task = std::make_unique<RotationOptimiser>();
            }
            else if (task_type == "multipath") {
                log<NUClear::INFO>("Task type is Multipath");
                task = std::make_unique<MultiPathOptimiser>();
            }
            else if (task_type == "stand") {
                log<NUClear::INFO>("Task type is Stand");
                task = std::make_unique<StandOptimiser>();
            }
            else {
                log<NUClear::ERROR>("Unrecognised optimiser task", task_type);
                powerplant.shutdown();
            }
            task->SetupNSGA2(config, nsga2_algorithm);
        });

        on<Startup>().then([this]() {
            // Create a message to request an evaluation of an individual
            std::unique_ptr<WebotsReady> message = std::make_unique<WebotsReady>();
            log<NUClear::INFO>("Starting up in 4 seconds");
            emit<Scope::DELAY>(message, std::chrono::seconds(4));
        });

        on<Trigger<WebotsReady>, Single>().then([this]() {
            log<NUClear::INFO>("webots ready, starting first evaluation");

            // If initialisation succeeded, evaluate the first individual of the first generation
            // Subsequent individuals will be evaluated after we get the evaluation scores for this
            // individual (from the NSGA2FitnessScores trigger)
            if (nsga2_algorithm.InitializeFirstGeneration()) {
                emit(std::make_unique<NSGA2EvaluatorReady>());
            }
            else {
                log<NUClear::ERROR>("Failed to initialise NSGA2");
            }
        });

        on<Trigger<NSGA2EvaluatorReady>, Single>().then([this]() {
            auto next_ind = nsga2_algorithm.getCurrentPop()->GetNextIndividual();
            if (next_ind.has_value()) {
                auto ind = next_ind.value();
                log<NUClear::INFO>("\n\nSending request to evaluator. Generation:",
                                   ind.generation,
                                   "individual",
                                   ind.id);
                // Create a message to request an evaluation of an individual
                emit(task->MakeEvaluationRequest(ind.id, ind.generation, ind.reals));
            }
            else {
                log<NUClear::INFO>("Evaluator ready, but optimiser is not");
                emit<Scope::DELAY>(std::make_unique<NSGA2EvaluatorReady>(),
                                   std::chrono::seconds(1));  // Wait for a bit for us to become ready, then
                                                              // ask the evaluator if it is ready
            }
        });

        on<Trigger<NSGA2FitnessScores>, Single>().then([this](const NSGA2FitnessScores& scores) {
            log<NUClear::DEBUG>("Got evaluation fitness scores", scores.obj_score[0], scores.obj_score[1]);

            // Tell the algorithm the evaluation scores for this individual
            nsga2_algorithm.getCurrentPop()->SetEvaluationResults(scores.id, scores.obj_score, scores.constraints);

            if (nsga2_algorithm.getCurrentPop()->AreAllEvaluated()) {
                // End the generation and save its data
                nsga2_algorithm.CompleteGenerationAndAdvance();

                if (nsga2_algorithm.HasMetOptimisationTerminalCondition()) {
                    log<NUClear::INFO>("NSGA2 evaluation finished!");

                    // Tell Webots to terminate
                    std::unique_ptr<OptimisationCommand> msg = std::make_unique<OptimisationCommand>();
                    msg->command                             = OptimisationCommand::CommandType::TERMINATE;
                    emit(msg);

                    // Tell the NSGA2 components to finish up, but add a delay to give webots time to get
                    // the terminate
                    emit<Scope::DELAY>(std::make_unique<NSGA2Terminate>(), std::chrono::milliseconds(100));
                }
                else {
                    log<NUClear::INFO>("Advanced to new generation", nsga2_algorithm.getCurrentPop()->generation);
                }
            }
            else {
                log<NUClear::DEBUG>("Recorded Evaluation for individual", scores.id, "more to come...");
            }
        });

        on<Trigger<NSGA2Terminate>, Single>().then([this]() {
            // NSGA2Terminate is emitted when we've finished all generations and all individuals
            // Tell ourselves to terminate
            log<NUClear::INFO>("Powerplant shutdown");
            powerplant.shutdown();
        });
    }
}  // namespace module::support::optimisation
