#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H

#include <nuclear>
#include <vector>

#include "nsga2.h"

namespace module {
namespace support {
namespace optimisation {

    class NSGA2Optimiser : public NUClear::Reactor {

        struct NSGA2EvaluationRequest
        {
            int id,
            int generation,
            std::vector<double> reals
            //std::vector<double> bins,
            //std::vector<std::vector<int>> gene
            //std::vector<double> objScore,
            //std::vector<double> constraints
        };

        struct NSGA2FitnessScores
        {
            int id,
            int generation,
            std::vector<double> objScore,
            std::vector<double> constraints
        };

    public:
        /// @brief Called by the powerplant to build and setup the NSGA2Optimiser reactor.
        explicit NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment);

        void fitnessFunction(int _id, int _generation, std::vector<double> _reals,
            std::vector<double> _bins, std::vector<std::vector<int>> _gene,
            std::vector<double> _objScore, std::vector<double> _constraints);

        nsga2::NSGA2 nsga2Algorithm;
        nsga2::RandomGenerator randGen;
    };

}
}
}

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_H
