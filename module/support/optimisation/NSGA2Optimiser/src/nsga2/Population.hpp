#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP

#include <algorithm>
#include <memory>

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
                   const bool& _crowdObj,
                   std::shared_ptr<RandomGenerator<>> _randGen,
                   const std::vector<double>& _initialRealVars);
        virtual ~Population() = default;
        void Initialize(const bool& randomInitialize);
        void Decode();
        void EvaluateInd(const int& _id);
        std::vector<double> GetIndReals(const int& _id);
        void SetIndObjectiveScore(const int& _id, const std::vector<double>& _objScore);
        void SetIndConstraints(const int& _id, const std::vector<double>& _constraints);
        void CheckConstraints();
        void FastNDS();
        void CrowdingDistanceAll();
        void CrowdingDistance(const int& _frontI);
        std::pair<int, int> Mutate();
        void Merge(const Population& _pop1, const Population& _pop2);
        void Report(std::ostream& _os) const;
        int GetSize() const {
            return int(inds.size());
        }

        std::vector<Individual> inds;
        int generation;
        std::vector<std::vector<int>> front = {};

    private:
        IndividualConfigurator indConfig;

        int size;
        bool crowdObj;

        friend std::ostream& operator<<(std::ostream& _os, const Population& _pop);
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP
