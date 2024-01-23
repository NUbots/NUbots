/* Adapted from https://github.com/dojeda/nsga2-cpp
 *
 * MIT License
 *
 * Copyright (c) 2021 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_NSGA2_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_NSGA2_HPP

#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "Population.hpp"
#include "Random.hpp"

namespace nsga2 {
    class NSGA2 {
    public:
        NSGA2() : rand_gen(std::make_shared<RandomGenerator<>>()) {}

        bool initialize_first_generation();
        void complete_generation_and_advance();
        void initialize_next_generation();
        bool has_met_optimisation_terminal_condition();

        // clang-format off
        void set_seed(const int& _seed) { rand_gen->set_seed(_seed); }
        void set_real_variable_count(const int& real_vars_) { real_vars = real_vars_; }
        void set_bin_variable_count(const int& bin_vars_) { bin_vars = bin_vars_; }
        void set_objective_count(const int& objectives_) { objectives = objectives_; }
        void set_constraint_count(const int& constraints_) { constraints = constraints_; }
        void set_population_size(const int& pop_size_) { pop_size = pop_size_; }
        void set_target_generations(const int& _generations) { generations = _generations; }
        void set_real_crossover_probability(const double& real_cross_prob_) { real_cross_prob = real_cross_prob_; }
        void set_bin_crossover_probability(const double& bin_cross_prob_) { bin_cross_prob = bin_cross_prob_; }
        void set_real_mutation_probability(const double& real_mut_prob_) { real_mut_prob = real_mut_prob_; }
        void set_bin_mutation_probability(const double& bin_mut_prob_) { bin_mut_prob = bin_mut_prob_; }
        void set_eta_c(const double& eta_c_) { etaC = eta_c_; }
        void set_eta_m(const double& eta_m_) { eta_m = eta_m_; }
        void set_eps_c(const double& eps_c_) { eps_c = eps_c_; }
        void set_bit_count(const std::vector<int>& bin_bits_) { bin_bits = bin_bits_; }
        void set_real_var_limits(const std::vector<std::pair<double, double>>& real_limits_) { real_limits = real_limits_; }
        void set_bin_var_limits(const std::vector<std::pair<double, double>>& bin_limits_) { bin_limits = bin_limits_; }
        void set_initial_real_vars(const std::vector<double>& init_real_vars_) { initial_real_vars = init_real_vars_; }
        void set_initial_population_real_vars(const std::vector<std::vector<double>>& supplied_population_real_vars_)
                                                { supplied_population_real_vars = supplied_population_real_vars_;}
        void set_supplied_pop(const bool& supplied_pop_) {supplied_pop = supplied_pop_;}
        // clang-format on

        std::shared_ptr<Population> get_current_pop();

        bool crowd_obj       = true;
        int report_count     = 0;
        int bin_mut_count    = 0;
        int real_mut_count   = 0;
        int bin_cross_count  = 0;
        int real_cross_count = 0;
        int bit_length       = 0;

        int current_gen;
        int pop_size;
        int generations;

    private:
        int real_vars             = -1;
        int bin_vars              = -1;
        int objectives            = -1;
        int constraints           = -1;
        double real_cross_prob    = -1.0;
        double bin_cross_prob     = -1.0;
        double real_mut_prob      = -1.0;
        double bin_mut_prob       = -1.0;
        double etaC               = -1.0;
        double eta_m              = -1.0;
        double eps_c              = std::numeric_limits<double>::epsilon();
        std::vector<int> bin_bits = {};
        std::vector<double> initial_real_vars;
        bool random_initialize                             = {};
        std::vector<std::pair<double, double>> real_limits = {};
        std::vector<std::pair<double, double>> bin_limits  = {};
        // Added for an intial population
        bool supplied_pop = false;
        std::vector<std::vector<double>> supplied_population_real_vars;

        std::shared_ptr<Population> parent_pop      = nullptr;
        std::shared_ptr<Population> child_pop       = nullptr;
        std::shared_ptr<Population> combined_pop    = nullptr;
        std::shared_ptr<RandomGenerator<>> rand_gen = nullptr;

        // Output file streams
        std::ofstream initial_pop_file;
        std::ofstream final_pop_file;
        std::ofstream all_pop_file;
        std::ofstream nsga2_params_file;

        // EarlyStopping
        bool early_stopping_one_front      = false;
        bool early_stopping_no_improvement = false;

        bool configuration_is_valid();
        void create_starting_populations();
        void initialize_reporting_streams();
        void write_report_headers(std::ofstream& _os, std::string file_name) const;
        void report_params(std::ofstream& _os, std::string file_name) const;
        void report_pop(const std::shared_ptr<Population>& pop, std::ofstream& os) const;

        void complete_generation();

        void selection(const std::shared_ptr<Population>& oldpop, std::shared_ptr<Population>& newpop);
        const Individual& tournament(const Individual& ind_1, const Individual& ind_2) const;
        void crossover(const Individual& parent_1,
                       const Individual& parent_2,
                       Individual& child_1,
                       Individual& child_2);
        void self_adaptive_sbx(const Individual& parent_1,
                               const Individual& parent_2,
                               Individual& child_1,
                               Individual& child_2);
        void bincross(const Individual& parent_1, const Individual& parent_2, Individual& child_1, Individual& child_2);
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_NSGA2_HPP
