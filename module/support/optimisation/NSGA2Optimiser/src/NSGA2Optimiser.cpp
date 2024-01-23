#include "NSGA2Optimiser.hpp"

#include "tasks/MultiPathOptimiser.hpp"
#include "tasks/RotationOptimiser.hpp"
#include "tasks/StrafeOptimiser.hpp"
#include "tasks/WalkOptimiser.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"
#include "message/support/optimisation/OptimisationCommand.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::support::optimisation {

    using extension::Configuration;

    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2EvaluatorReady;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2Terminate;
    using message::support::optimisation::OptimisationCommand;

    using utility::support::Expression;

    NSGA2Optimiser::NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Read NSGA2Optimiser.yaml file and initialize the values we're going to use for the optimisation
        on<Configuration>("NSGA2Optimiser.yaml").then([this](const Configuration& config) {
            log<NUClear::INFO>("Trying to setup NSGA2");

            // Set up the NSGA2 algorithm based on our config values
            nsga2_algorithm.set_objective_count(config["num_objectives"].as<int>());
            nsga2_algorithm.set_constraint_count(config["num_constraints"].as<int>());
            nsga2_algorithm.set_population_size(config["population_size"].as<int>());
            nsga2_algorithm.set_target_generations(config["max_generations"].as<int>());

            nsga2_algorithm.set_real_crossover_probability(config["probabilities"]["real"]["crossover"].as<double>());
            nsga2_algorithm.set_bin_crossover_probability(config["probabilities"]["binary"]["crossover"].as<double>());
            nsga2_algorithm.set_real_mutation_probability(config["probabilities"]["real"]["mutation"].as<double>());
            nsga2_algorithm.set_bin_mutation_probability(config["probabilities"]["binary"]["mutation"].as<double>());
            nsga2_algorithm.set_eta_c(config["eta"]["C"].as<double>());
            nsga2_algorithm.set_eta_m(config["eta"]["M"].as<double>());
            nsga2_algorithm.set_seed(config["seed"].as<int>());

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
            else {
                log<NUClear::ERROR>("Unrecognised optimiser task", task_type);
                powerplant.shutdown();
            }
            task->setup_nsga2(config, nsga2_algorithm);
        });

        on<Startup>().then([this]() {
            // Create a message to request an evaluation of an individual
            log<NUClear::INFO>("Starting up in 4 seconds");
            std::this_thread::sleep_for(std::chrono::seconds(4));

            log<NUClear::INFO>("Optimiser ready, starting first evaluation");

            // If initialisation succeeded, evaluate the first individual of the first generation
            // Subsequent individuals will be evaluated after we get the evaluation scores for this
            // individual (from the NSGA2FitnessScores trigger)
            if (nsga2_algorithm.initialize_first_generation()) {
                emit(std::make_unique<NSGA2EvaluatorReady>());
            }
            else {
                log<NUClear::ERROR>("Failed to initialise NSGA2");
            }
        });

        on<Trigger<NSGA2EvaluatorReady>, Single>().then([this]() {
            auto next_ind = nsga2_algorithm.get_current_pop()->get_next_individual();
            if (next_ind.has_value()) {
                auto ind = next_ind.value();
                log<NUClear::INFO>("\n\nSending request to evaluator. Generation:",
                                   ind.generation,
                                   "individual",
                                   ind.id);
                // Create a message to request an evaluation of an individual
                emit(task->make_evaluation_request(ind.id, ind.generation, ind.reals));
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
            nsga2_algorithm.get_current_pop()->set_evaluation_results(scores.id, scores.obj_score, scores.constraints);

            if (nsga2_algorithm.get_current_pop()->are_all_evaluated()) {
                // End the generation and save its data
                nsga2_algorithm.complete_generation_and_advance();

                if (nsga2_algorithm.has_met_optimisation_terminal_condition()) {
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
                    log<NUClear::INFO>("Advanced to new generation", nsga2_algorithm.get_current_pop()->generation);
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
