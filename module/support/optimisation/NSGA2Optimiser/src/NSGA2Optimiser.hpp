#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H

#include <nuclear>
#include <vector>

#include "nsga2/NSGA2.hpp"

#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::support::optimisation::NSGA2EvaluationRequest;

            class NSGA2Optimiser : public NUClear::Reactor {
            private:
                /// @brief Create a message for the Evaluator module to evaluate the given individual in the given
                /// generation. id is the individual's id (a number, starts at 0 for first individual in the
                /// generation) generation is the generation number, starts at 1 for the first generation parameters
                /// are the individual's parameters to optimise,
                std::unique_ptr<NSGA2EvaluationRequest> constructEvaluationRequest(int id, int generation, const std::vector<double>& parameters);

                void populateEvaluationRequests();
                bool atEndOfGeneration();

                void processOrdinaryGenerationIndividual(int id, int generation, const std::vector<double>& objScore, const std::vector<double>& constraints);
                void processFinalGenerationIndividual(int id, int generation, const std::vector<double>& objScore, const std::vector<double>& constraints);

                /// @brief Implementation of the NSGA II algorithm, holds the state of the entire optimisation,
                /// including the populations, scores, etc
                nsga2::NSGA2 nsga2Algorithm{};

                /// @brief Default leg gains for the walk, read from the config file
                double default_leg_gains;

                /// @brief Queue of evaluation requests waiting to be processed
                std::queue<std::unique_ptr<NSGA2EvaluationRequest>> pendingEvaluationRequests;
                std::unordered_multiset<int> runningEvaluationRequests;

            public:
                /// @brief Called by the powerplant to build and setup the NSGA2Optimiser reactor.
                explicit NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
