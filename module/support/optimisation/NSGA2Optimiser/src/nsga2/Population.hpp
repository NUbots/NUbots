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
        Population(const int& _size,
                   const int& _realVars,
                   const int& _binVars,
                   const int& _constraints,
                   const std::vector<int>& _binBits,
                   const std::vector<std::pair<double, double>>& _realLimits,
                   const std::vector<std::pair<double, double>>& _binLimits,
                   const int& _objectives,
                   const double& _realMutProb,
                   const double& _binMutProb,
                   const double& _etaM,
                   const double& _epsC,
                   std::shared_ptr<RandomGenerator<>> _randGen,
                   const std::vector<double>& _initialRealVars);
        virtual ~Population() = default;

        bool initialised = false; //So we know when the population is ready to evaluate (otherwise we can start evaluating before the individuals are initialised)
        int generation = -1;

        void resetCurrentIndividualIndex();
        void SetIndividualsGeneration(const int _generation);
        std::optional<Individual> GetNextIndividual();

        void SetIds();

        void Initialize();
        void Decode();

        int GetSize() const {
            return int(inds.size());
        }

        bool IsReadyToEvalulate() const;

        bool AreAllEvaluated() const;
        void SetEvaluationResults(const int& _id, const std::vector<double>& _objScore, const std::vector<double>& _constraints);

        void FastNDS();
        void CrowdingDistanceAll();
        void CrowdingDistance(const int& _frontIndex);

        std::pair<int, int> Mutate();

        void Merge(const Population& _pop1, const Population& _pop2);
        void Report(std::ostream& _os, int currentGen) const;

        std::vector<Individual> inds;

        std::vector<std::vector<int>> fronts = {};

    private:
        std::size_t currentInd = 0;
        IndividualConfigurator indConfig;
        int size;
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP
