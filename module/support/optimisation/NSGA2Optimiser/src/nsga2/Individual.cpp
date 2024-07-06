/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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

#include "Individual.hpp"

#include <cmath>
#include <fmt/format.h>
#include <nuclear>

namespace nsga2 {
    Individual::Individual(const IndividualConfigurator& _config)
        : rank(0), constr_violation(0.0), crowd_dist(0.0), evaluated(false), id(0), generation(-1), config(_config) {
        reals.clear();
        gene.clear();
        bins.clear();
        obj_score.clear();
        constr.clear();

        reals.resize(config.real_vars, 0);
        bins.resize(config.bin_vars, 0);
        gene.resize(config.bin_vars);

        if (int(config.bin_bits.size()) != config.bin_vars) {
            NUClear::log<NUClear::WARN>(
                fmt::format("bin_bits size ({}) != bin_vars ({})", config.bin_bits.size(), config.bin_vars));
        }

        for (int j = 0; j < config.bin_vars; j++) {
            gene[j].resize(config.bin_bits[j], 0);
        }
        obj_score.resize(config.objectives, 0);
        constr.resize(config.constraints, 0);
    }

    void Individual::initialize(const int& _id) {
        id = _id;

        if (id == 0) {
            // First individual gets real vars from config
            // initialise real vars
            for (int i = 0; i < config.real_vars; i++) {
                reals[i] = config.initial_real_vars[i];
            }
        }
        else {
            // initialise real vars
            for (int i = 0; i < config.real_vars; i++) {
                reals[i] = config.rand_gen->Real(config.real_limits[i].first, config.real_limits[i].second);
            }
        }
        // initialise bin vars
        for (int i = 0; i < config.bin_vars; i++) {
            for (int j = 0; j < config.bin_bits[i]; j++) {
                gene[i][j] = config.rand_gen->Realu() <= 0.5 ? 0 : 1;
            }
        }
    }

    void Individual::decode() {
        int sum;
        for (int i = 0; i < config.bin_vars; i++) {
            sum = 0;
            for (int j = 0; j < config.bin_bits[i]; j++) {
                sum += (1 << (config.bin_bits[i] - 1 - j));
            }

            bins[i] = config.bin_limits[i].first
                      + (double) sum * (config.bin_limits[i].second - config.bin_limits[i].first)
                            / (double) ((1 << (config.bin_bits[i])) - 1);
        }
    }

    void Individual::check_constraints() {
        if (config.constraints) {
            constr_violation = 0.0;
            for (int i = 0; i < config.constraints; i++) {
                if (constr[i] < 0.0)
                    constr_violation += constr[i];
            }
        }
        else
            constr_violation = 0.0;

        evaluated = true;
    }

    // returns:  1 if this < _b (this dominates _b),
    //          -1 if this > _b (this is dominated by _b),
    //           0 if they are nondominated
    int Individual::check_dominance(const Individual& _b) const {
        if (constr_violation < 0.0 && _b.constr_violation < 0.0) {
            // both have constraint violations
            if (constr_violation > _b.constr_violation)
                return 1;  // this violates less
            else if (constr_violation < _b.constr_violation)
                return -1;  // _b violates less
            else
                return 0;  // they both violate equally
        }
        else if (constr_violation < 0 && _b.constr_violation == 0) {
            // this violates and _b doesn't => _b dominates
            return -1;
        }
        else if (constr_violation == 0 && _b.constr_violation < 0) {
            // this doesn't violate and _b does => this dominates
            return 1;
        }
        else {
            // no constraint violations
            int this_doms = 0;  // to check if this has a smaller objective
            int that_doms = 0;  // to check if _b    has a smaller objective

            for (int i = 0; i < config.objectives; i++) {
                if (config.objectives > 1) {
                    // Normal multi objective comparison
                    if (obj_score[i] < _b.obj_score[i])
                        this_doms = 1;
                    else if (obj_score[i] > _b.obj_score[i])
                        that_doms = 1;
                }
                else {
                    // mono objective comparison with an epsilon
                    double obj_fabs = std::fabs(obj_score[i] - _b.obj_score[i]);
                    if (obj_score[i] < _b.obj_score[i] && obj_fabs > config.eps_c)
                        this_doms = 1;
                    else if (obj_score[i] > _b.obj_score[i] && obj_fabs > config.eps_c)
                        that_doms = 1;
                }
            }

            // there is at least one smaller objective for this and none for _b
            if (this_doms == 1 && that_doms == 0)
                return 1;
            // there is at least one smaller objective for _b and none for this
            else if (this_doms == 0 && that_doms == 1)
                return -1;
            // no smaller objective or both have one smaller
            else
                return 0;
        }
    }

    std::pair<int, int> Individual::mutate() {
        std::pair<int, int> mutation_count = std::make_pair(0, 0);
        if (config.real_vars) {
            mutation_count.first += real_mutate();
        }
        if (config.bin_vars) {
            mutation_count.second += bin_mutate();
        }
        return mutation_count;
    }

    int Individual::real_mutate() {
        int mutation_count = 0;
        for (int i = 0; i < config.real_vars; i++) {
            if (config.rand_gen->Realu() <= config.real_mut_prob) {
                double real_i       = reals[i];
                double real_i_lower = config.real_limits[i].first;
                double real_i_upper = config.real_limits[i].second;
                double delta_1      = (real_i - real_i_lower) / (real_i_upper - real_i_lower);
                double delta_2      = (real_i_upper - real_i) / (real_i_upper - real_i_lower);
                double rand_real    = config.rand_gen->Realu();
                double mut_power    = 1.0 / (config.eta_m + 1.0);

                double delta_inverse, value, delta_q;
                if (rand_real <= 0.5) {
                    delta_inverse = 1.0 - delta_1;
                    value = 2.0 * rand_real + (1.0 - 2.0 * rand_real) * (std::pow(delta_inverse, (config.eta_m + 1.0)));
                    delta_q = std::pow(value, mut_power) - 1.0;
                }
                else {
                    delta_inverse = 1.0 - delta_2;
                    value         = 2.0 * (1.0 - rand_real)
                            + 2.0 * (rand_real - 0.5) * (std::pow(delta_inverse, (config.eta_m + 1.0)));
                    delta_q = 1.0 - (std::pow(value, mut_power));
                }

                real_i = real_i + delta_q * (real_i_upper - real_i_lower);

                if (real_i < real_i_lower)
                    real_i = real_i_lower;
                if (real_i > real_i_upper)
                    real_i = real_i_upper;
                reals[i] = real_i;

                mutation_count++;
            }
        }
        return mutation_count;
    }

    int Individual::bin_mutate() {
        int mutation_count = 0;
        for (int i = 0; i < config.bin_vars; i++) {
            for (int j = 0; j < config.bin_bits[i]; j++) {
                double probability = config.rand_gen->Realu();
                if (probability <= config.bin_mut_prob) {
                    if (gene[i][j] == 0)
                        gene[i][j] = 1;
                    else
                        gene[i][j] = 0;
                    mutation_count++;
                }
            }
        }
        return mutation_count;
    }

    void Individual::report(std::ostream& os, int population_generation) const {
        os << population_generation << "," << generation << "," << id << "," << constr_violation << "," << rank << ","
           << crowd_dist;

        for (int i = 0; i < config.objectives; i++) {
            os << "," << obj_score[i];
        }

        for (int i = 0; i < config.constraints; i++) {
            os << "," << constr[i];
        }

        for (int i = 0; i < config.real_vars; i++) {
            os << "," << reals[i];
        }

        for (int i = 0; i < config.bin_vars; i++) {
            for (int j = 0; j < config.bin_bits[i]; j++) {
                os << "," << gene[i][j];
            }
        }

        os << std::endl;
    }
}  // namespace nsga2
