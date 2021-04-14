#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H

#include <nuclear>
#include <vector>

#include "extension/Script.h"
#include "nsga2.h"

namespace module {
namespace support {
    namespace optimisation {

        class NSGA2Optimiser : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the NSGA2Optimiser reactor.
            explicit NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment);

            void requestIndEvaluation(int _id, int _generation, const std::vector<double>& _reals);

            nsga2::NSGA2 nsga2Algorithm;
            nsga2::RandomGenerator randGen;
        };

    }  // namespace optimisation
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
