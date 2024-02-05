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
        void initialize(const int& _id);
        void decode();
        std::pair<int, int> mutate();
        int check_dominance(const Individual& _b) const;
        void check_constraints();

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

        void report(std::ostream& _os, int population_generation) const;
        int id;
        int generation;

    private:
        std::vector<double> bins;
        int real_mutate();
        int bin_mutate();
        IndividualConfigurator config;
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_INDIVIDUAL_HPP
