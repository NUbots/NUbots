#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H

#include <nuclear>
#include <vector>

#include "nsga2/NSGA2.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            class NSGA2Optimiser : public NUClear::Reactor {
            private:
                /// @brief Send a message to the Evaluator module to evaluate the given individual in the given
                /// generation. _id is the individual's id (a number, starts at 0 for first individual in the
                /// generation) _generation is the generation number, starts at 1 for the first generation _reals are
                /// the individual's parameters to optimise,
                void requestIndEvaluation(int _id, int _generation, const std::vector<double>& _reals);

                /// @brief Implementation of the NSGA II algorithm, holds the state of the entire optimisation,
                /// including the populations, scores, etc
                nsga2::NSGA2 nsga2Algorithm;

                /// @brief Default leg gains for the walk, read from the config file
                double default_leg_gains;

            public:
                /// @brief Called by the powerplant to build and setup the NSGA2Optimiser reactor.
                explicit NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
