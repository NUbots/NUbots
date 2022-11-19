#include "NSGA2.hpp"

#include <cmath>
#include <nuclear>

namespace nsga2 {
    struct sort_n {
        // Implementation of operator <n, used to choose the best individuals from a front
        const Population& pop;
        sort_n(const Population& population) : pop(population) {}
        bool operator()(int i, int j) {
            const Individual& ind1 = pop.inds[i];
            const Individual& ind2 = pop.inds[j];
            if (ind1.rank < ind2.rank)
                return true;
            else if (ind1.rank == ind2.rank && ind1.crowdDist > ind2.crowdDist)
                return true;
            return false;
        }
    };

    bool NSGA2::ConfigurationIsValid() {
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
        else if (etaM <= 0) {
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

    void NSGA2::CreateStartingPopulations() {
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
                                                 etaM,
                                                 epsC,
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
                                                etaM,
                                                epsC,
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
                                                   etaM,
                                                   epsC,
                                                   rand_gen,
                                                   initial_real_vars);
    }

    std::shared_ptr<Population> NSGA2::getCurrentPop() {
        if (current_gen == 0) {
            return parent_pop;
        }
        else {
            return child_pop;
        }
    }

    bool NSGA2::InitializeFirstGeneration() {
        if (!ConfigurationIsValid()) {
            return false;
        }

        InitializeReportingStreams();

        bin_mut_count    = 0;
        real_mut_count   = 0;
        bin_cross_count  = 0;
        real_cross_count = 0;

        bit_length = std::accumulate(bin_bits.begin(), bin_bits.end(), 0);

        CreateStartingPopulations();
        current_gen = 0;
        parent_pop->Initialize();

        if(supplied_pop){
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
        parent_pop->SetIndividualsGeneration(current_gen);
        parent_pop->Decode();
        parent_pop->initialised = true;
        return true;
    }

    void NSGA2::CompleteGenerationAndAdvance() {
        child_pop->initialised = false;  // Stop all evaluation while we work out the next childPop
        CompleteGeneration();
        if (HasMetOptimisationTerminalCondition()) {
            // Report the population of our final generation
            ReportPop(parent_pop, final_pop_file);
        }
        else {
            // Start the next generation (creates the new children for evaluation)
            InitializeNextGeneration();
        }
    }


    bool NSGA2::HasMetOptimisationTerminalCondition() {
        return (current_gen >= generations) || (earlyStoppingNoImprovement && earlyStoppingOneFront);
    }

    void NSGA2::CompleteGeneration() {
        if (current_gen == 0) {
            // In the first generation, we don't have to deal with a previous parent generation
            parent_pop->FastNDS();              // Calculate the fronts
            parent_pop->CrowdingDistanceAll();  // Calculate the crowding distance of the fronts

            ReportPop(parent_pop, initial_pop_file);
        }
        else {
            combined_pop->Merge(
                *parent_pop,
                *child_pop);          // Create combined population from parent and child populations. Rt = Pt U Qt
            combined_pop->FastNDS();  // Calculate the fronts

            parent_pop->inds.clear();  // Empty the new parent population, ready to be repopulated

            int i = 0;  // we need `i` after the loop, for the final (partial) front, so hold on to it here
            while (parent_pop->GetSize() + int(combined_pop->fronts[i].size())
                   < pop_size) {  // stop when adding the next front would go past the population size
                std::vector<int>& front_i = combined_pop->fronts[i];
                combined_pop->CrowdingDistance(i);  // calculate crowding in front_i
                for (std::size_t j = 0; j < front_i.size(); j++) {
                    // Include the i-th non-dominated front in the parent pop. i.e. Pt+1 = Pt+1 U Fi
                    parent_pop->inds.push_back(combined_pop->inds[front_i[j]]);
                }
                i++;
            }
            // At this point, we don't have space to add the next full front, so we add the best members from that front
            std::vector<int>& partial_front = combined_pop->fronts[i];
            combined_pop->CrowdingDistance(i);  // calculate crowding in front_i, so that we can choose the best members

            std::sort(partial_front.begin(),
                      partial_front.end(),
                      sort_n(*combined_pop));  // sort remaining front using operator <n

            const int remainingSpace = pop_size - parent_pop->GetSize();
            for (int j = 0; j < remainingSpace; j++) {
                // Include the best members from the remaining front in the next parent population
                parent_pop->inds.push_back(combined_pop->inds[partial_front[j]]);
            }

            // Early stopping checks
            if (i == 0) {
                // If `i` is still 0, that means we only have one front carrying over
                earlyStoppingOneFront = true;
                NUClear::log<NUClear::INFO>("A single front this generation, could stop early");
            }
            bool aChildSurvivesThisGen = false;
            for (auto& ind : parent_pop->inds) {
                if (ind.generation == current_gen) {
                    aChildSurvivesThisGen = true;
                    break;
                }
            }
            if (!aChildSurvivesThisGen) {
                earlyStoppingNoImprovement = true;
                NUClear::log<NUClear::INFO>("No improvement this generation, could stop early");
            }
        }
        ReportPop(parent_pop, all_pop_file);
        current_gen++;
        parent_pop->generation = current_gen;
    }

    void NSGA2::InitializeNextGeneration() {
        // create next child population, Q_t
        Selection(parent_pop, child_pop);
        std::pair<int, int> mutationsCount = child_pop->Mutate();
        // mutation book-keeping
        real_mut_count += mutationsCount.first;
        bin_mut_count  += mutationsCount.second;

        child_pop->SetIndividualsGeneration(current_gen);
        child_pop->SetIds();
        child_pop->Decode();
        child_pop->initialised = true;
        child_pop->resetCurrentIndividualIndex();
    }

    // Selection implements the tournament and crossover steps of generating the new pop
    void NSGA2::Selection(const std::shared_ptr<Population>& _oldPop, std::shared_ptr<Population>& _newPop) {
        const int oldPopSize = _oldPop->GetSize();
        if (_newPop->GetSize() != oldPopSize) {
            NUClear::log<NUClear::ERROR>("Selection error: new and old pops don't have the same size");
        }

        // Set up lists, ready for random scrambling
        std::vector<int> indList1(oldPopSize), indList2(oldPopSize);
        for (int i = 0; i < oldPopSize; i++) {
            indList1[i] = i;
            indList2[i] = i;
        }

        // Create random pairings
        for (int i = 0; i < oldPopSize; i++) {
            int randInt = rand_gen->Integer(i, oldPopSize - 1);
            std::swap(indList1[randInt], indList1[i]);
            randInt = rand_gen->Integer(i, oldPopSize - 1);
            std::swap(indList2[randInt], indList2[i]);
        }

        // Tournament to select the best parents from the pairings, then crossover to combine their params
        for (int i = 0; i < oldPopSize; i += 4) {
            const Individual& p11 = Tournament(_oldPop->inds[indList1[i]], _oldPop->inds[indList1[i + 1]]);
            const Individual& p12 = Tournament(_oldPop->inds[indList1[i + 2]], _oldPop->inds[indList1[i + 3]]);
            Crossover(p11, p12, _newPop->inds[i], _newPop->inds[i + 1]);

            const Individual& p21 = Tournament(_oldPop->inds[indList2[i]], _oldPop->inds[indList2[i + 1]]);
            const Individual& p22 = Tournament(_oldPop->inds[indList2[i + 2]], _oldPop->inds[indList2[i + 3]]);
            Crossover(p21, p22, _newPop->inds[i + 2], _newPop->inds[i + 3]);
        }
    }

    // Tournament decides which individual is allowed to reproduce
    const Individual& NSGA2::Tournament(const Individual& _ind1, const Individual& _ind2) const {
        const int comparison = _ind1.CheckDominance(_ind2);
        if (comparison == 1) {  // ind1 dominates ind2
            return _ind1;
        }
        else if (comparison == -1) {  // ind2 dominates ind1
            return _ind2;
        }
        else if (_ind1.crowdDist > _ind2.crowdDist) {
            return _ind1;
        }
        else if (_ind2.crowdDist > _ind1.crowdDist) {
            return _ind2;
        }
        else if (rand_gen->Realu() <= 0.5) {
            return _ind1;
        }
        else {
            return _ind2;
        }
    }

    // Mix the parameters of the parents
    void NSGA2::Crossover(const Individual& _parent1,
                          const Individual& _parent2,
                          Individual& _child1,
                          Individual& _child2) {
        if (real_vars) {
            SelfAdaptiveSBX(_parent1, _parent2, _child1, _child2);
        }
        if (bin_vars) {
            Bincross(_parent1, _parent2, _child1, _child2);
        }

        _child1.evaluated = false;
        _child2.evaluated = false;
    }

    // Self Adaptive Simulated Binary Crossover (SBX) is a particular implementation of crossover
    void NSGA2::SelfAdaptiveSBX(const Individual& _parent1,
                                const Individual& _parent2,
                                Individual& _child1,
                                Individual& _child2) {
        double y1, y2, yLower, yUpper;
        double c1, c2;
        double alpha, beta, betaQ;

        if (rand_gen->Realu() <= real_cross_prob) {  // If we should crossover (determined by RNG)
            real_cross_count++;                     // Keep track of the number of times we do crossover
            for (int i = 0; i < real_vars; i++) {  // for each real parameter
                // If the parameters are different
                if (std::fabs(_parent1.reals[i] - _parent2.reals[i]) > std::numeric_limits<double>::epsilon()) {
                    // hold the smaller param in y1, the larger param in y2
                    if (_parent1.reals[i] < _parent2.reals[i]) {
                        y1 = _parent1.reals[i];
                        y2 = _parent2.reals[i];
                    }
                    else {
                        y1 = _parent2.reals[i];
                        y2 = _parent1.reals[i];
                    }

                    // keep track of the parameter limits
                    yLower = real_limits[i].first;
                    yUpper = real_limits[i].second;

                    double randDouble = rand_gen->Realu();
                    beta              = 1.0 + (2.0 * (y1 - yLower) / (y2 - y1));
                    alpha             = 2.0 - std::pow(beta, -(etaC + 1.0));
                    if (randDouble <= (1.0 / alpha))
                        betaQ = std::pow((randDouble * alpha), (1.0 / (etaC + 1.0)));
                    else
                        betaQ = std::pow((1.0 / (2.0 - randDouble * alpha)), (1.0 / (etaC + 1.0)));

                    c1    = 0.5 * ((y1 + y2) - betaQ * (y2 - y1));
                    beta  = 1.0 + (2.0 * (yUpper - y2) / (y2 - y1));
                    alpha = 2.0 - std::pow(beta, -(etaC + 1.0));

                    if (randDouble <= (1.0 / alpha))
                        betaQ = std::pow((randDouble * alpha), (1.0 / (etaC + 1.0)));
                    else
                        betaQ = std::pow((1.0 / (2.0 - randDouble * alpha)), (1.0 / (etaC + 1.0)));

                    c2 = 0.5 * ((y1 + y2) + betaQ * (y2 - y1));
                    c1 = std::min(std::max(c1, yLower), yUpper);
                    c2 = std::min(std::max(c2, yLower), yUpper);

                    if (rand_gen->Realu() <= 0.5) {
                        _child1.reals[i] = c2;
                        _child2.reals[i] = c1;
                    }
                    else {
                        _child1.reals[i] = c1;
                        _child2.reals[i] = c2;
                    }
                }
                else {  // If the parameters are the same, then we can just copy them
                    _child1.reals[i] = _parent1.reals[i];
                    _child2.reals[i] = _parent2.reals[i];
                }
            }
        }
        else {  // If we shoudn't crossover (determined by RNG)
            for (int i = 0; i < real_vars; i++) {
                _child1.reals[i] = _parent1.reals[i];
                _child2.reals[i] = _parent2.reals[i];
            }
        }
    }

    void NSGA2::Bincross(const Individual& _parent1,
                         const Individual& _parent2,
                         Individual& _child1,
                         Individual& _child2) {
        int temp, site1, site2;
        for (int i = 0; i < bin_vars; i++) {
            double randDouble = rand_gen->Realu();
            if (randDouble <= bin_cross_prob) {
                bin_cross_count++;
                site1 = rand_gen->Integer(0, bin_bits[i] - 1);
                site2 = rand_gen->Integer(0, bin_bits[i] - 1);
                if (site1 > site2) {
                    temp  = site1;
                    site1 = site2;
                    site2 = temp;
                }

                for (int j = 0; j < site1; j++) {
                    _child1.gene[i][j] = _parent1.gene[i][j];
                    _child2.gene[i][j] = _parent2.gene[i][j];
                }
                for (int j = site1; j < site2; j++) {
                    _child1.gene[i][j] = _parent2.gene[i][j];
                    _child2.gene[i][j] = _parent1.gene[i][j];
                }
                for (int j = site2; j < bin_bits[i]; j++) {
                    _child1.gene[i][j] = _parent1.gene[i][j];
                    _child2.gene[i][j] = _parent2.gene[i][j];
                }
            }
            else {
                for (int j = 0; j < bin_bits[i]; j++) {
                    _child1.gene[i][j] = _parent1.gene[i][j];
                    _child2.gene[i][j] = _parent2.gene[i][j];
                }
            }
        }
    }

    void NSGA2::InitializeReportingStreams() {
        WriteReportHeaders(initial_pop_file, "nsga2_initial_pop.csv");
        WriteReportHeaders(final_pop_file, "nsga2_final_pop.csv");
        WriteReportHeaders(all_pop_file, "nsga2_all_pop.csv");

        ReportParams(nsga2_params_file, "nsga2_params.csv");
    }

    void NSGA2::WriteReportHeaders(std::ofstream& _os, std::string file_name) const {
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

    void NSGA2::ReportPop(const std::shared_ptr<Population>& _pop, std::ofstream& _os) const {
        _pop->Report(_os, current_gen);
        _os.flush();
    }

    void NSGA2::ReportParams(std::ofstream& _os, std::string file_name) const {
        _os.open(file_name, std::ios::out | std::ios::trunc);
        _os.precision(16);

        _os << "# General Parameters\n";
        _os << "population_size,num_generations,num_objective_functions,num_constraints,rand_seed"
            << ",num_real_vars,real_var_crossover_prob,real_var_mutation_prob"
            << ",crossover_dist_index,mutation_dist_index"
            << ",num_binary_vars,binary_var_crossover_prob,binary_var_mutation_prob" << std::endl;

        _os << pop_size << "," << generations << "," << objectives << "," << constraints << "," << rand_gen->GetSeed()
            << "," << real_vars << "," << real_cross_prob << "," << real_mut_prob << "," << etaC << "," << etaM << ","
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
                _os << i << "," << bin_limits[i].first << "," << bin_limits[i].second << "," << bin_bits[i] << std::endl;
            }
        }
        _os.flush();
    }
}  // namespace nsga2
