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
        void Initialize(const int& _id);
        void Decode();
        std::pair<int, int> Mutate();
        int CheckDominance(const Individual& _b) const;
        void CheckConstraints();

        int rank;
        double constrViolation;
        std::vector<std::vector<int>> gene;
        std::vector<double> reals; //Real Parameters
        std::vector<double> objScore; //Evaluation score
        std::vector<double> constr; //Constraint violations
        std::vector<int> dominationList; //S_p, the set of individuals that this individual domintates
        int dominatedByCounter; //n_p, the number of individuals that dominate this individual
        double crowdDist; //Crowding distance, i.e. how close is the next nearest solution. Boundary solutions have infinite distance.
        bool evaluated;

        void Report(std::ostream& _os, int population_generation) const;
        int id;
        int generation;

    private:
        std::vector<double> bins;
        int realMutate();
        int binMutate();
        IndividualConfigurator config;
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_INDIVIDUAL_HPP
