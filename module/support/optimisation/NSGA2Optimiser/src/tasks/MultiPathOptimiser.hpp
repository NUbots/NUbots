#ifndef MODULE_SUPPORT_OPTIMISATION_MULTIPATH_H
#define MODULE_SUPPORT_OPTIMISATION_MULTIPATH_H

#include <nuclear>
#include <vector>

#include "OptimiserTask.hpp"
#include "nsga2/NSGA2.hpp"

#include "extension/Configuration.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::support::optimisation::NSGA2EvaluationRequest;

            class MultiPathOptimiser : public OptimiserTask {
            public:
                void SetupNSGA2(const ::extension::Configuration& config, nsga2::NSGA2& nsga2Algorithm);
                std::unique_ptr<NSGA2EvaluationRequest> MakeEvaluationRequest(const int id,
                                                                              const int generation,
                                                                              std::vector<double> reals);
                std::vector<std::vector<double>> readInPopulationFile(const std::string flie_name);

            private:
                int trial_duration_limit;
                std::string quintic_walk_path;
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_MULTIPATH_H
