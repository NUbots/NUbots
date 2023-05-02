#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP

#include <algorithm>
#include <memory>
#include <optional>

#include "Individual.hpp"
#include "Random.hpp"

namespace nsga2 {
    class Population {
    public:
        Population(const int& size_,
                   const int& realVars_,
                   const int& binVars_,
                   const int& constraints_,
                   const std::vector<int>& binBits_,
                   const std::vector<std::pair<double, double>>& realLimits_,
                   const std::vector<std::pair<double, double>>& binLimits_,
                   const int& objectives_,
                   const double& realMutProb_,
                   const double& binMutProb_,
                   const double& etaM_,
                   const double& epsC_,
                   std::shared_ptr<RandomGenerator<>> randGen_,
                   const std::vector<double>& initialRealVars_);
        virtual ~Population() = default;

        bool initialised = false;  // So we know when the population is ready to evaluate (otherwise we can start
                                   // evaluating before the individuals are initialised)
        int generation = -1;

        void resetCurrentIndividualIndex();
        void SetIndividualsGeneration(const int generation_);
        std::optional<Individual> GetNextIndividual();

        void SetIds();

        void Initialize();
        void Decode();

        int GetSize() const {
            return int(inds.size());
        }

        bool IsReadyToEvalulate() const;

        bool AreAllEvaluated() const;
        void SetEvaluationResults(const int& _id,
                                  const std::vector<double>& objScore_,
                                  const std::vector<double>& constraints_);

        void FastNDS();
        void CrowdingDistanceAll();
        void CrowdingDistance(const int& frontIndex_);

        std::pair<int, int> Mutate();

        void Merge(const Population& pop1_, const Population& pop2_);
        void Report(std::ostream& os_, int currentGen) const;

        std::vector<Individual> inds;  // Got to extend class

        std::vector<std::vector<int>> fronts = {};

    private:
        std::size_t currentInd = 0;
        IndividualConfigurator indConfig;
        int size;
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP
