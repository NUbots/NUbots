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
                   const int& real_vars_,
                   const int& bin_vars_,
                   const int& constraints_,
                   const std::vector<int>& bin_bits_,
                   const std::vector<std::pair<double, double>>& real_limits_,
                   const std::vector<std::pair<double, double>>& bin_limits_,
                   const int& objectives_,
                   const double& real_mut_prob_,
                   const double& bin_mut_prob_,
                   const double& eta_m_,
                   const double& eps_c_,
                   std::shared_ptr<RandomGenerator<>> rand_gen_,
                   const std::vector<double>& initial_real_vars_);
        virtual ~Population() = default;

        bool initialised = false;  // So we know when the population is ready to evaluate (otherwise we can start
                                   // evaluating before the individuals are initialised)
        int generation = -1;

        void reset_current_individual_index();
        void set_individuals_generation(const int generation_);
        std::optional<Individual> get_next_individual();

        void set_ids();

        void initialize();
        void decode();

        int get_size() const {
            return int(inds.size());
        }

        bool is_ready_to_evalulate() const;

        bool are_all_evaluated() const;
        void set_evaluation_results(const int& _id,
                                    const std::vector<double>& obj_score_,
                                    const std::vector<double>& constraints_);

        void fast_nds();
        void crowding_distance_all();
        void crowding_distance(const int& front_index_);

        std::pair<int, int> mutate();

        void merge(const Population& pop_1_, const Population& pop_2_);
        void report(std::ostream& os_, int current_gen) const;

        std::vector<Individual> inds;  // Got to extend class

        std::vector<std::vector<int>> fronts = {};

    private:
        std::size_t current_ind = 0;
        IndividualConfigurator ind_config;
        int size;
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_POPULATION_HPP
