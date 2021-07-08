#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_INDIVIDUAL_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_INDIVIDUAL_HPP

#include <memory>
#include <ostream>
#include <vector>

#include "Random.hpp"

namespace nsga2 {
    struct IndividualConfigurator {
        int realVars;
        std::vector<std::pair<double, double>> realLimits;
        double realMutProb;
        int binVars;
        std::vector<int> binBits;
        std::vector<std::pair<double, double>> binLimits;
        double binMutProb;
        int objectives;
        int constraints;
        double etaM;
        double epsC;
        std::shared_ptr<RandomGenerator<>> randGen;
        std::vector<double> initialRealVars;
    };

    class Individual {
    public:
        Individual(const IndividualConfigurator& _config);
        virtual ~Individual() = default;
        void Initialize(const int& _id, const bool& randomInitialize);
        void Decode();
        std::pair<int, int> Mutate();
        int CheckDominance(const Individual& _b) const;
        void CheckConstraints();

        int rank;
        double constrViolation;
        std::vector<std::vector<int>> gene;
        std::vector<double> reals;
        std::vector<double> objScore;
        std::vector<double> constr;
        std::vector<int> dominationList; //S_p, the set of individuals that this individual domintates
        int dominatedByCounter; //n_p, the number of individuals that dominate this individual
        double crowdDist;
        bool evaluated;

        // Public for report access
        int id;
        int generation;

    private:
        std::vector<double> bins;
        int realMutate();
        int binMutate();
        IndividualConfigurator config;

        friend std::ostream& operator<<(std::ostream& _os, const Individual& _ind);
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_INDIVIDUAL_HPP
