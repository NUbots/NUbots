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

#include "NSGA2.hpp"

#include <cmath>
#include <nuclear>

namespace nsga2 {
    struct sort_n {
        // Implementation of operator <n, used to choose the best individuals from a front
        const Population& pop;
        sort_n(const Population& population) : pop(population) {}
        bool operator()(int i, int j) {
            const Individual& ind_1 = pop.inds[i];
            const Individual& ind_2 = pop.inds[j];
            if (ind_1.rank < ind_2.rank)
                return true;
            else if (ind_1.rank == ind_2.rank && ind_1.crowd_dist > ind_2.crowd_dist)
                return true;
            return false;
        }
    };

    bool NSGA2::configuration_is_valid() {
        NUClear::log<NUClear::INFO>("Checking NSGA-II configuration...");

        if (real_vars < 0) {
            NUClear::log<NUClear::INFO>("Invalid number of real variables");
            return false;
        }
        else if (bin_vars < 0) {
            NUClear::log<NUClear::INFO>("Invalid number of binary variables");
            return false;
        }
        else if (real_vars == 0 && bin_vars == 0) {
            NUClear::log<NUClear::INFO>("Zero real and binary variables");
            return false;
        }
        else if (objectives < 1) {
            NUClear::log<NUClear::INFO>("Invalid number of objective functions");
            return false;
        }
        else if (constraints < 0) {
            NUClear::log<NUClear::INFO>("Invalid number of constraints");
            return false;
        }
        else if (pop_size < 4 || (pop_size % 4) != 0) {
            NUClear::log<NUClear::INFO>("Invalid size of population");
            return false;
        }
        else if (real_cross_prob < 0.0 || real_cross_prob > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of real crossover");
            return false;
        }
        else if (real_mut_prob < 0.0 || real_mut_prob > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of real mutation");
            return false;
        }
        else if (bin_cross_prob < 0.0 || bin_cross_prob > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of binary crossover");
            return false;
        }
        else if (bin_mut_prob < 0.0 || bin_mut_prob > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of binary mutation");
            return false;
        }
        else if (etaC <= 0) {
            NUClear::log<NUClear::INFO>("Invalid distribution index for crossover");
            return false;
        }
        else if (eta_m <= 0) {
            NUClear::log<NUClear::INFO>("Invalid distribution index for mutation");
            return false;
        }
        else if (generations < 1) {
            NUClear::log<NUClear::INFO>("Invalid number of generations");
            return false;
        }
        else if (bin_vars != 0 && bin_bits.size() == 0) {
            NUClear::log<NUClear::INFO>("Invalid number of bits for binary variables");
            return false;
        }
        else if (int(real_limits.size()) != real_vars) {
            NUClear::log<NUClear::INFO>("Invalid number of real variable limits");
            return false;
        }
        else if (int(bin_limits.size()) != bin_vars) {
            NUClear::log<NUClear::INFO>("Invalid number of binary variable limits");
            return false;
        }
        else if ((int(initial_real_vars.size()) != real_vars)) {
            NUClear::log<NUClear::INFO>("Invalid number of initial real variables");
            return false;
        }
        else {
            NUClear::log<NUClear::INFO>("NSGA-II configuration is valid");
            return true;
        }
    }

    void NSGA2::create_starting_populations() {
        parent_pop = std::make_shared<Population>(pop_size,
                                                  real_vars,
                                                  bin_vars,
                                                  constraints,
                                                  bin_bits,
                                                  real_limits,
                                                  bin_limits,
                                                  objectives,
                                                  real_mut_prob,
                                                  bin_mut_prob,
                                                  eta_m,
                                                  eps_c,
                                                  rand_gen,
                                                  initial_real_vars);

        child_pop = std::make_shared<Population>(pop_size,
                                                 real_vars,
                                                 bin_vars,
                                                 constraints,
                                                 bin_bits,
                                                 real_limits,
                                                 bin_limits,
                                                 objectives,
                                                 real_mut_prob,
                                                 bin_mut_prob,
                                                 eta_m,
                                                 eps_c,
                                                 rand_gen,
                                                 initial_real_vars);

        combined_pop = std::make_shared<Population>(pop_size * 2,
                                                    real_vars,
                                                    bin_vars,
                                                    constraints,
                                                    bin_bits,
                                                    real_limits,
                                                    bin_limits,
                                                    objectives,
                                                    real_mut_prob,
                                                    bin_mut_prob,
                                                    eta_m,
                                                    eps_c,
                                                    rand_gen,
                                                    initial_real_vars);
    }

    std::shared_ptr<Population> NSGA2::get_current_pop() {
        if (current_gen == 0) {
            return parent_pop;
        }
        else {
            return child_pop;
        }
    }

    bool NSGA2::initialize_first_generation() {
        if (!configuration_is_valid()) {
            return false;
        }

        initialize_reporting_streams();

        bin_mut_count    = 0;
        real_mut_count   = 0;
        bin_cross_count  = 0;
        real_cross_count = 0;

        bit_length = std::accumulate(bin_bits.begin(), bin_bits.end(), 0);

        create_starting_populations();
        current_gen = 0;
        parent_pop->initialize();

        if (supplied_pop) {
            std::vector<double> reals;
            for (size_t i = 1; i < parent_pop->inds.size(); i++) {
                reals = parent_pop->inds[i].reals;
                for (size_t j = 0; j < reals.size(); j++) {
                    reals[j] = supplied_population_real_vars[i][j];
                }
                parent_pop->inds[i].reals = reals;
            }
        }

        parent_pop->generation = current_gen;
        parent_pop->set_individuals_generation(current_gen);
        parent_pop->decode();
        parent_pop->initialised = true;
        return true;
    }

    void NSGA2::complete_generation_and_advance() {
        child_pop->initialised = false;  // Stop all evaluation while we work out the next childPop
        complete_generation();
        if (has_met_optimisation_terminal_condition()) {
            // Report the population of our final generation
            report_pop(parent_pop, final_pop_file);
        }
        else {
            // Start the next generation (creates the new children for evaluation)
            initialize_next_generation();
        }
    }


    bool NSGA2::has_met_optimisation_terminal_condition() {
        return (current_gen >= generations) || (early_stopping_no_improvement && early_stopping_one_front);
    }

    void NSGA2::complete_generation() {
        if (current_gen == 0) {
            // In the first generation, we don't have to deal with a previous parent generation
            parent_pop->fast_nds();               // Calculate the fronts
            parent_pop->crowding_distance_all();  // Calculate the crowding distance of the fronts

            report_pop(parent_pop, initial_pop_file);
        }
        else {
            combined_pop->merge(
                *parent_pop,
                *child_pop);           // Create combined population from parent and child populations. Rt = Pt U Qt
            combined_pop->fast_nds();  // Calculate the fronts

            parent_pop->inds.clear();  // Empty the new parent population, ready to be repopulated

            int i = 0;  // we need `i` after the loop, for the final (partial) front, so hold on to it here
            while (parent_pop->get_size() + int(combined_pop->fronts[i].size())
                   < pop_size) {  // stop when adding the next front would go past the population size
                std::vector<int>& front_i = combined_pop->fronts[i];
                combined_pop->crowding_distance(i);  // calculate crowding in front_i
                for (std::size_t j = 0; j < front_i.size(); j++) {
                    // Include the i-th non-dominated front in the parent pop. i.e. Pt+1 = Pt+1 U Fi
                    parent_pop->inds.push_back(combined_pop->inds[front_i[j]]);
                }
                i++;
            }
            // At this point, we don't have space to add the next full front, so we add the best members from that front
            std::vector<int>& partial_front = combined_pop->fronts[i];
            combined_pop->crowding_distance(
                i);  // calculate crowding in front_i, so that we can choose the best members

            std::sort(partial_front.begin(),
                      partial_front.end(),
                      sort_n(*combined_pop));  // sort remaining front using operator <n

            const int remaining_space = pop_size - parent_pop->get_size();
            for (int j = 0; j < remaining_space; j++) {
                // Include the best members from the remaining front in the next parent population
                parent_pop->inds.push_back(combined_pop->inds[partial_front[j]]);
            }

            // Early stopping checks
            if (i == 0) {
                // If `i` is still 0, that means we only have one front carrying over
                early_stopping_one_front = true;
                NUClear::log<NUClear::INFO>("A single front this generation, could stop early");
            }
            bool a_child_survives_this_gen = false;
            for (auto& ind : parent_pop->inds) {
                if (ind.generation == current_gen) {
                    a_child_survives_this_gen = true;
                    break;
                }
            }
            if (!a_child_survives_this_gen) {
                early_stopping_no_improvement = true;
                NUClear::log<NUClear::INFO>("No improvement this generation, could stop early");
            }
        }
        report_pop(parent_pop, all_pop_file);
        current_gen++;
        parent_pop->generation = current_gen;
    }

    void NSGA2::initialize_next_generation() {
        // create next child population, Q_t
        selection(parent_pop, child_pop);
        std::pair<int, int> mutations_count = child_pop->mutate();
        // mutation book-keeping
        real_mut_count += mutations_count.first;
        bin_mut_count += mutations_count.second;

        child_pop->set_individuals_generation(current_gen);
        child_pop->set_ids();
        child_pop->decode();
        child_pop->initialised = true;
        child_pop->reset_current_individual_index();
    }

    // selection implements the tournament and crossover steps of generating the new pop
    void NSGA2::selection(const std::shared_ptr<Population>& _old_pop, std::shared_ptr<Population>& _new_pop) {
        const int old_pop_size = _old_pop->get_size();
        if (_new_pop->get_size() != old_pop_size) {
            NUClear::log<NUClear::ERROR>("selection error: new and old pops don't have the same size");
        }

        // Set up lists, ready for random scrambling
        std::vector<int> ind_list_1(old_pop_size), ind_list_2(old_pop_size);
        for (int i = 0; i < old_pop_size; i++) {
            ind_list_1[i] = i;
            ind_list_2[i] = i;
        }

        // Create random pairings
        for (int i = 0; i < old_pop_size; i++) {
            int rand_int = rand_gen->Integer(i, old_pop_size - 1);
            std::swap(ind_list_1[rand_int], ind_list_1[i]);
            rand_int = rand_gen->Integer(i, old_pop_size - 1);
            std::swap(ind_list_2[rand_int], ind_list_2[i]);
        }

        // tournament to select the best parents from the pairings, then crossover to combine their params
        for (int i = 0; i < old_pop_size; i += 4) {
            const Individual& p11 = tournament(_old_pop->inds[ind_list_1[i]], _old_pop->inds[ind_list_1[i + 1]]);
            const Individual& p12 = tournament(_old_pop->inds[ind_list_1[i + 2]], _old_pop->inds[ind_list_1[i + 3]]);
            crossover(p11, p12, _new_pop->inds[i], _new_pop->inds[i + 1]);

            const Individual& p21 = tournament(_old_pop->inds[ind_list_2[i]], _old_pop->inds[ind_list_2[i + 1]]);
            const Individual& p22 = tournament(_old_pop->inds[ind_list_2[i + 2]], _old_pop->inds[ind_list_2[i + 3]]);
            crossover(p21, p22, _new_pop->inds[i + 2], _new_pop->inds[i + 3]);
        }
    }

    // tournament decides which individual is allowed to reproduce
    const Individual& NSGA2::tournament(const Individual& _ind_1, const Individual& _ind_2) const {
        const int comparison = _ind_1.check_dominance(_ind_2);
        if (comparison == 1) {  // ind_1 dominates ind_2
            return _ind_1;
        }
        else if (comparison == -1) {  // ind_2 dominates ind_1
            return _ind_2;
        }
        else if (_ind_1.crowd_dist > _ind_2.crowd_dist) {
            return _ind_1;
        }
        else if (_ind_2.crowd_dist > _ind_1.crowd_dist) {
            return _ind_2;
        }
        else if (rand_gen->Realu() <= 0.5) {
            return _ind_1;
        }
        else {
            return _ind_2;
        }
    }

    // Mix the parameters of the parents
    void NSGA2::crossover(const Individual& _parent_1,
                          const Individual& _parent_2,
                          Individual& _child_1,
                          Individual& _child_2) {
        if (real_vars) {
            self_adaptive_sbx(_parent_1, _parent_2, _child_1, _child_2);
        }
        if (bin_vars) {
            bincross(_parent_1, _parent_2, _child_1, _child_2);
        }

        _child_1.evaluated = false;
        _child_2.evaluated = false;
    }

    // Self Adaptive Simulated Binary crossover (SBX) is a particular implementation of crossover
    void NSGA2::self_adaptive_sbx(const Individual& _parent_1,
                                  const Individual& _parent_2,
                                  Individual& _child_1,
                                  Individual& _child_2) {
        double y1, y2, y_lower, y_upper;
        double c1, c2;
        double alpha, beta, beta_q;

        if (rand_gen->Realu() <= real_cross_prob) {  // If we should crossover (determined by RNG)
            real_cross_count++;                      // Keep track of the number of times we do crossover
            for (int i = 0; i < real_vars; i++) {    // for each real parameter
                // If the parameters are different
                if (std::fabs(_parent_1.reals[i] - _parent_2.reals[i]) > std::numeric_limits<double>::epsilon()) {
                    // hold the smaller param in y1, the larger param in y2
                    if (_parent_1.reals[i] < _parent_2.reals[i]) {
                        y1 = _parent_1.reals[i];
                        y2 = _parent_2.reals[i];
                    }
                    else {
                        y1 = _parent_2.reals[i];
                        y2 = _parent_1.reals[i];
                    }

                    // keep track of the parameter limits
                    y_lower = real_limits[i].first;
                    y_upper = real_limits[i].second;

                    double rand_double = rand_gen->Realu();
                    beta               = 1.0 + (2.0 * (y1 - y_lower) / (y2 - y1));
                    alpha              = 2.0 - std::pow(beta, -(etaC + 1.0));
                    if (rand_double <= (1.0 / alpha))
                        beta_q = std::pow((rand_double * alpha), (1.0 / (etaC + 1.0)));
                    else
                        beta_q = std::pow((1.0 / (2.0 - rand_double * alpha)), (1.0 / (etaC + 1.0)));

                    c1    = 0.5 * ((y1 + y2) - beta_q * (y2 - y1));
                    beta  = 1.0 + (2.0 * (y_upper - y2) / (y2 - y1));
                    alpha = 2.0 - std::pow(beta, -(etaC + 1.0));

                    if (rand_double <= (1.0 / alpha))
                        beta_q = std::pow((rand_double * alpha), (1.0 / (etaC + 1.0)));
                    else
                        beta_q = std::pow((1.0 / (2.0 - rand_double * alpha)), (1.0 / (etaC + 1.0)));

                    c2 = 0.5 * ((y1 + y2) + beta_q * (y2 - y1));
                    c1 = std::min(std::max(c1, y_lower), y_upper);
                    c2 = std::min(std::max(c2, y_lower), y_upper);

                    if (rand_gen->Realu() <= 0.5) {
                        _child_1.reals[i] = c2;
                        _child_2.reals[i] = c1;
                    }
                    else {
                        _child_1.reals[i] = c1;
                        _child_2.reals[i] = c2;
                    }
                }
                else {  // If the parameters are the same, then we can just copy them
                    _child_1.reals[i] = _parent_1.reals[i];
                    _child_2.reals[i] = _parent_2.reals[i];
                }
            }
        }
        else {  // If we shoudn't crossover (determined by RNG)
            for (int i = 0; i < real_vars; i++) {
                _child_1.reals[i] = _parent_1.reals[i];
                _child_2.reals[i] = _parent_2.reals[i];
            }
        }
    }

    void NSGA2::bincross(const Individual& _parent_1,
                         const Individual& _parent_2,
                         Individual& _child_1,
                         Individual& _child_2) {
        int temp, site_1, site_2;
        for (int i = 0; i < bin_vars; i++) {
            double rand_double = rand_gen->Realu();
            if (rand_double <= bin_cross_prob) {
                bin_cross_count++;
                site_1 = rand_gen->Integer(0, bin_bits[i] - 1);
                site_2 = rand_gen->Integer(0, bin_bits[i] - 1);
                if (site_1 > site_2) {
                    temp   = site_1;
                    site_1 = site_2;
                    site_2 = temp;
                }

                for (int j = 0; j < site_1; j++) {
                    _child_1.gene[i][j] = _parent_1.gene[i][j];
                    _child_2.gene[i][j] = _parent_2.gene[i][j];
                }
                for (int j = site_1; j < site_2; j++) {
                    _child_1.gene[i][j] = _parent_2.gene[i][j];
                    _child_2.gene[i][j] = _parent_1.gene[i][j];
                }
                for (int j = site_2; j < bin_bits[i]; j++) {
                    _child_1.gene[i][j] = _parent_1.gene[i][j];
                    _child_2.gene[i][j] = _parent_2.gene[i][j];
                }
            }
            else {
                for (int j = 0; j < bin_bits[i]; j++) {
                    _child_1.gene[i][j] = _parent_1.gene[i][j];
                    _child_2.gene[i][j] = _parent_2.gene[i][j];
                }
            }
        }
    }

    void NSGA2::initialize_reporting_streams() {
        write_report_headers(initial_pop_file, "nsga2_initial_pop.csv");
        write_report_headers(final_pop_file, "nsga2_final_pop.csv");
        write_report_headers(all_pop_file, "nsga2_all_pop.csv");

        report_params(nsga2_params_file, "nsga2_params.csv");
    }

    void NSGA2::write_report_headers(std::ofstream& _os, std::string file_name) const {
        _os.open(file_name, std::ios::out | std::ios::trunc);
        _os.precision(16);

        _os << "population_generation"
            << ","
            << "individual_generation"
            << ","
            << "individual"
            << ","
            << "constraint_violation"
            << ","
            << "rank"
            << ","
            << "crowding_dist";

        for (int i = 0; i < objectives; i++) {
            _os << ","
                << "objective_" << i;
        }

        for (int i = 0; i < constraints; i++) {
            _os << ","
                << "constraints_" << i;
        }

        for (int i = 0; i < real_vars; i++) {
            _os << ","
                << "real_param_" << i;
        }

        for (int i = 0; i < bin_vars; i++) {
            for (int j = 0; j < bin_bits[i]; j++) {
                _os << ","
                    << "binary_param_" << i << "_" << j;
            }
        }

        _os << std::endl;
        _os.flush();
    }

    void NSGA2::report_pop(const std::shared_ptr<Population>& _pop, std::ofstream& _os) const {
        _pop->report(_os, current_gen);
        _os.flush();
    }

    void NSGA2::report_params(std::ofstream& _os, std::string file_name) const {
        _os.open(file_name, std::ios::out | std::ios::trunc);
        _os.precision(16);

        _os << "# General Parameters\n";
        _os << "population_size,num_generations,num_objective_functions,num_constraints,rand_seed"
            << ",num_real_vars,real_var_crossover_prob,real_var_mutation_prob"
            << ",crossover_dist_index,mutation_dist_index"
            << ",num_binary_vars,binary_var_crossover_prob,binary_var_mutation_prob" << std::endl;

        _os << pop_size << "," << generations << "," << objectives << "," << constraints << "," << rand_gen->get_seed()
            << "," << real_vars << "," << real_cross_prob << "," << real_mut_prob << "," << etaC << "," << eta_m << ","
            << "," << bin_vars << "," << bin_cross_prob << "," << bin_mut_prob << std::endl;


        if (real_vars > 0) {
            _os << "\n";

            _os << "# Real Variable Parameters\n";
            _os << "index,initial_value,lower_limit,upper_limit" << std::endl;

            for (int i = 0; i < real_vars; i++) {
                _os << i << "," << initial_real_vars[i] << "," << real_limits[i].first << "," << real_limits[i].second
                    << std::endl;
            }
        }

        if (bin_vars > 0) {
            _os << "\n";

            _os << "# Binary Variable Parameters\n";
            _os << "index,lower_limit,upper_limit,num_bits" << std::endl;

            for (int i = 0; i < bin_vars; i++) {
                _os << i << "," << bin_limits[i].first << "," << bin_limits[i].second << "," << bin_bits[i]
                    << std::endl;
            }
        }
        _os.flush();
    }
}  // namespace nsga2
