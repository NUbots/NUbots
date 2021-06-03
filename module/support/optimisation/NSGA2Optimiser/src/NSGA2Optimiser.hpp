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
                void requestIndEvaluation(int _id, int _generation, const std::vector<double>& _reals);

                nsga2::NSGA2 nsga2Algorithm;

                double default_gain;

            public:
                /// @brief Called by the powerplant to build and setup the NSGA2Optimiser reactor.
                explicit NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
