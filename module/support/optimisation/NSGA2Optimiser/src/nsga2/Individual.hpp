#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_INDIVIDUAL_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_INDIVIDUAL_HPP

#include <memory>
#include <ostream>
#include <vector>

#include "Random.hpp"

namespace nsga2 {
    struct IndividualConfigurator {
        int real_vars;
        std::vector<std::pair<double, double>> real_limits;
        double real_mut_prob;
        int bin_vars;
        std::vector<int> bin_bits;
        std::vector<std::pair<double, double>> bin_limits;
        double bin_mut_prob;
        int objectives;
        int constraints;
        double eta_m;
        double eps_c;
        std::shared_ptr<RandomGenerator<>> rand_gen;
        std::vector<double> initial_real_vars;
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
        double constr_violation;
        std::vector<std::vector<int>> gene;
        std::vector<double> reals;         // Real Parameters
        std::vector<double> obj_score;     // Evaluation score
        std::vector<double> constr;        // Constraint violations
        std::vector<int> domination_list;  // S_p, the set of individuals that this individual domintates
        int dominated_by_counter;          // n_p, the number of individuals that dominate this individual
        double crowd_dist;  // Crowding distance, i.e. how close is the next nearest solution. Boundary solutions have
                            // infinite distance.
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
