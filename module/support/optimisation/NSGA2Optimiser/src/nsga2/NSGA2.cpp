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
        if (realVars < 0) {
            NUClear::log<NUClear::INFO>("Invalid number of real variables");
            return false;
        } else if (binVars < 0) {
            NUClear::log<NUClear::INFO>("Invalid number of binary variables");
            return false;
        } else if (realVars == 0 && binVars == 0) {
            NUClear::log<NUClear::INFO>("Zero real and binary variables");
            return false;
        } else if (objectives < 1) {
            NUClear::log<NUClear::INFO>("Invalid number of objective functions");
            return false;
        } else if (constraints < 0) {
            NUClear::log<NUClear::INFO>("Invalid number of constraints");
            return false;
        } else if (popSize < 4 || (popSize % 4) != 0) {
            NUClear::log<NUClear::INFO>("Invalid size of population");
            return false;
        } else if (realCrossProb < 0.0 || realCrossProb > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of real crossover");
            return false;
        } else if (realMutProb < 0.0 || realMutProb > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of real mutation");
            return false;
        } else if (binCrossProb < 0.0 || binCrossProb > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of binary crossover");
            return false;
        } else if (binMutProb < 0.0 || binMutProb > 1.0) {
            NUClear::log<NUClear::INFO>("Invalid probability of binary mutation");
            return false;
        } else if (etaC <= 0) {
            NUClear::log<NUClear::INFO>("Invalid distribution index for crossover");
            return false;
        } else if (etaM <= 0) {
            NUClear::log<NUClear::INFO>("Invalid distribution index for mutation");
            return false;
        } else if (generations < 1) {
            NUClear::log<NUClear::INFO>("Invalid number of generations");
            return false;
        } else if (binVars != 0 && binBits.size() == 0) {
            NUClear::log<NUClear::INFO>("Invalid number of bits for binary variables");
            return false;
        } else if (int(realLimits.size()) != realVars) {
            NUClear::log<NUClear::INFO>("Invalid number of real variable limits");
            return false;
        } else if (int(binLimits.size()) != binVars) {
            NUClear::log<NUClear::INFO>("Invalid number of binary variable limits");
            return false;
        } else if ((int(initialRealVars.size()) != realVars)) {
            NUClear::log<NUClear::INFO>("Invalid number of initial real variables");
            return false;
        } else {
            NUClear::log<NUClear::INFO>("NSGA-II configuration is valid");
            return true;
        }
    }

    void NSGA2::CreateStartingPopulations() {
        parentPop = std::make_shared<Population>(popSize,
                                                 realVars,
                                                 binVars,
                                                 constraints,
                                                 binBits,
                                                 realLimits,
                                                 binLimits,
                                                 objectives,
                                                 realMutProb,
                                                 binMutProb,
                                                 etaM,
                                                 epsC,
                                                 randGen,
                                                 initialRealVars);

        childPop = std::make_shared<Population>(popSize,
                                                realVars,
                                                binVars,
                                                constraints,
                                                binBits,
                                                realLimits,
                                                binLimits,
                                                objectives,
                                                realMutProb,
                                                binMutProb,
                                                etaM,
                                                epsC,
                                                randGen,
                                                initialRealVars);

        combinedPop = std::make_shared<Population>(popSize * 2,
                                                realVars,
                                                binVars,
                                                constraints,
                                                binBits,
                                                realLimits,
                                                binLimits,
                                                objectives,
                                                realMutProb,
                                                binMutProb,
                                                etaM,
                                                epsC,
                                                randGen,
                                                initialRealVars);
    }

    std::shared_ptr<Population> NSGA2::getCurrentPop() {
        if(currentGen == 0) {
            return parentPop;
        } else {
            return childPop;
        }
    }

    bool NSGA2::InitializeFirstGeneration() {
        if(!ConfigurationIsValid()) {
            return false;
        }

        InitializeReportingStreams();
        ReportParams(nsga2_params_file);

        binMutCount    = 0;
        realMutCount   = 0;
        binCrossCount  = 0;
        realCrossCount = 0;

        bitLength = std::accumulate(binBits.begin(), binBits.end(), 0);

        CreateStartingPopulations();
        currentGen = 0;
        parentPop->Initialize();
        parentPop->generation = currentGen;
        parentPop->SetIndividualsGeneration(currentGen);
        parentPop->Decode();
        parentPop->initialised = true;
        return true;
    }

    void NSGA2::CompleteGenerationAndAdvance() {
        childPop->initialised = false; //Stop all evaluation while we work out the next childPop
        CompleteGeneration();
        if(HasMetOptimisationTerminalCondition()) {
            // Report the population of our final generation
            ReportPop(parentPop, final_pop_file);
        } else {
            // Start the next generation (creates the new children for evaluation)
            InitializeNextGeneration();
        }
    }


    bool NSGA2::HasMetOptimisationTerminalCondition() {
        return (currentGen >= generations) || earlyStoppingNoImprovement || earlyStoppingOneFront;
    }

    void NSGA2::CompleteGeneration() {
        if(currentGen == 0) {
            //In the first generation, we don't have to deal with a previous parent generation
            parentPop->FastNDS(); //Calculate the fronts
            parentPop->CrowdingDistanceAll(); //Calculate the crowding distance of the fronts

            ReportPop(parentPop, initial_pop_file);
        } else {
            combinedPop->Merge(*parentPop, *childPop); // Create combined population from parent and child populations. Rt = Pt U Qt
            combinedPop->FastNDS(); //Calculate the fronts

            parentPop->inds.clear(); //Empty the new parent population, ready to be repopulated

            int i = 0; //we need `i` after the loop, for the final (partial) front, so hold on to it here
            while (parentPop->GetSize() + int(combinedPop->fronts[i].size()) < popSize) { // stop when adding the next front would go past the population size
                std::vector<int>& front_i = combinedPop->fronts[i];
                combinedPop->CrowdingDistance(i);         // calculate crowding in front_i
                for (std::size_t j = 0; j < front_i.size(); j++) {
                    // Include the i-th non-dominated front in the parent pop. i.e. Pt+1 = Pt+1 U Fi
                    parentPop->inds.push_back(combinedPop->inds[front_i[j]]);
                }
                i++;
            }
            //At this point, we don't have space to add the next full front, so we add the best members from that front
            std::vector<int>& partial_front = combinedPop->fronts[i];
            combinedPop->CrowdingDistance(i);  // calculate crowding in front_i, so that we can choose the best members

            std::sort(partial_front.begin(), partial_front.end(), sort_n(*combinedPop));  // sort remaining front using operator <n

            const int remainingSpace = popSize - parentPop->GetSize();
            for (int j = 0; j < remainingSpace; j++) {
                // Include the best members from the remaining front in the next parent population
                parentPop->inds.push_back(combinedPop->inds[partial_front[j]]);
            }

            // Early stopping checks
            if(i == 0) {
                // If `i` is still 0, that means we only have one front carrying over
                earlyStoppingOneFront = true;
                NUClear::log<NUClear::INFO>("A single front this generation, stopping early");
            }
            bool aChildSurvivesThisGen = false;
            for (auto& ind : parentPop->inds) {
                if(ind.generation == currentGen) {
                    aChildSurvivesThisGen = true;
                    break;
                }
            }
            if(!aChildSurvivesThisGen) {
                earlyStoppingNoImprovement = true;
                NUClear::log<NUClear::INFO>("No improvement this generation, stopping early");
            }
        }
        ReportPop(parentPop, all_pop_file);
        currentGen++;
        parentPop->generation = currentGen;
    }

    void NSGA2::InitializeNextGeneration() {
        // create next child population, Q_t
        Selection(parentPop, childPop);
        std::pair<int, int> mutationsCount = childPop->Mutate();
        // mutation book-keeping
        realMutCount += mutationsCount.first;
        binMutCount += mutationsCount.second;

        childPop->SetIndividualsGeneration(currentGen);
        childPop->SetIds();
        childPop->Decode();
        childPop->initialised = true;
        childPop->resetCurrentIndividualIndex();
    }

    void NSGA2::Selection(const std::shared_ptr<Population>& _oldPop, std::shared_ptr<Population>& _newPop) {
        const int oldPopSize = _oldPop->GetSize();
        if (_newPop->GetSize() != oldPopSize) {
            NUClear::log<NUClear::ERROR>("Selection error: new and old pops don't have the same size");
        }

        std::vector<int> indList1(oldPopSize), indList2(oldPopSize);
        for (int i = 0; i < oldPopSize; i++) {
            indList1[i] = i;
            indList2[i] = i;
        }

        for (int i = 0; i < oldPopSize; i++) {
            int randInt = randGen->Integer(i, oldPopSize - 1);
            std::swap(indList1[randInt], indList1[i]);
            randInt = randGen->Integer(i, oldPopSize - 1);
            std::swap(indList2[randInt], indList2[i]);
        }

        for (int i = 0; i < oldPopSize; i += 4) {
            const Individual& p11 = Tournament(_oldPop->inds[indList1[i]], _oldPop->inds[indList1[i + 1]]);
            const Individual& p12 = Tournament(_oldPop->inds[indList1[i + 2]], _oldPop->inds[indList1[i + 3]]);
            Crossover(p11, p12, _newPop->inds[i], _newPop->inds[i + 1]);

            const Individual& p21 = Tournament(_oldPop->inds[indList2[i]], _oldPop->inds[indList2[i + 1]]);
            const Individual& p22 = Tournament(_oldPop->inds[indList2[i + 2]], _oldPop->inds[indList2[i + 3]]);
            Crossover(p21, p22, _newPop->inds[i + 2], _newPop->inds[i + 3]);
        }
    }

    const Individual& NSGA2::Tournament(const Individual& _ind1, const Individual& _ind2) const {
        const int comparison = _ind1.CheckDominance(_ind2);
        if (comparison == 1) {  // ind1 dominates ind2
            return _ind1;
        } else if (comparison == -1) {  // ind2 dominates ind1
            return _ind2;
        } else if (_ind1.crowdDist > _ind2.crowdDist) {
            return _ind1;
        } else if (_ind2.crowdDist > _ind1.crowdDist) {
            return _ind2;
        } else if (randGen->Realu() <= 0.5) {
            return _ind1;
        } else {
            return _ind2;
        }
    }

    void NSGA2::Crossover(const Individual& _parent1,
                          const Individual& _parent2,
                          Individual& _child1,
                          Individual& _child2) {
        if (realVars) {
            Realcross(_parent1, _parent2, _child1, _child2);
        }
        if (binVars) {
            Bincross(_parent1, _parent2, _child1, _child2);
        }

        _child1.evaluated = false;
        _child2.evaluated = false;
    }

    void NSGA2::Realcross(const Individual& _parent1,
                          const Individual& _parent2,
                          Individual& _child1,
                          Individual& _child2) {
        double y1, y2, yLower, yUpper;
        double c1, c2;
        double alpha, beta, betaQ;

        if (randGen->Realu() <= realCrossProb) {
            realCrossCount++;
            for (int i = 0; i < realVars; i++) {
                if (std::fabs(_parent1.reals[i] - _parent2.reals[i]) > std::numeric_limits<double>::epsilon()) {
                    if (_parent1.reals[i] < _parent2.reals[i]) {
                        y1 = _parent1.reals[i];
                        y2 = _parent2.reals[i];
                    }
                    else {
                        y1 = _parent2.reals[i];
                        y2 = _parent1.reals[i];
                    }

                    yLower = realLimits[i].first;
                    yUpper = realLimits[i].second;

                    double randDouble = randGen->Realu();
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

                    if (randGen->Realu() <= 0.5) {
                        _child1.reals[i] = c2;
                        _child2.reals[i] = c1;
                    } else {
                        _child1.reals[i] = c1;
                        _child2.reals[i] = c2;
                    }
                } else {
                    _child1.reals[i] = _parent1.reals[i];
                    _child2.reals[i] = _parent2.reals[i];
                }
            }
        } else {
            for (int i = 0; i < realVars; i++) {
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
        for (int i = 0; i < binVars; i++) {
            double randDouble = randGen->Realu();
            if (randDouble <= binCrossProb) {
                binCrossCount++;
                site1 = randGen->Integer(0, binBits[i] - 1);
                site2 = randGen->Integer(0, binBits[i] - 1);
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
                for (int j = site2; j < binBits[i]; j++) {
                    _child1.gene[i][j] = _parent1.gene[i][j];
                    _child2.gene[i][j] = _parent2.gene[i][j];
                }
            } else {
                for (int j = 0; j < binBits[i]; j++) {
                    _child1.gene[i][j] = _parent1.gene[i][j];
                    _child2.gene[i][j] = _parent2.gene[i][j];
                }
            }
        }
    }

    void NSGA2::InitializeReportingStreams() {
        InitializePopulationStream(initial_pop_file, "nsga2_initial_pop.csv", "This file contains the data of the initial generation");
        InitializePopulationStream(final_pop_file, "nsga2_final_pop.csv", "This file contains the data of the final population");
        InitializePopulationStream(best_pop_file, "nsga2_best_pop.csv", "This file contains the data of final feasible population (if any)");
        InitializePopulationStream(all_pop_file, "nsga2_all_pop.csv", "This file contains the data of all generations");

        nsga2_params_file.open("nsga2_params.csv", std::ios::out | std::ios::trunc);
        nsga2_params_file.precision(16);
    }

    void NSGA2::InitializePopulationStream(std::ofstream& file_stream, std::string file_name, std::string description) {
        file_stream.open(file_name, std::ios::out | std::ios::trunc);
        file_stream.precision(16);

        file_stream << "# " << description << "\n";
        file_stream << "# num_objectives: " << objectives << ", num_constraints: " << constraints
                    << ", num_real_variables: " << realVars << ", num_binary_variables: " << bitLength << std::endl;
    }

    void NSGA2::ReportParams(std::ostream& _os) const {
        _os << "# General Parameters\n";
        _os << "population_size,num_generations,num_objective_functions,num_constraints,rand_seed"
            << ",num_real_vars,real_var_crossover_prob,real_var_mutation_prob"
            << ",crossover_dist_index,mutation_dist_index"
            << ",num_binary_vars,binary_var_crossover_prob,binary_var_mutation_prob" << std::endl;

        _os << popSize << "," << generations << "," << objectives << "," << constraints << "," << randGen->GetSeed()
            << "," << realVars << "," << realCrossProb << "," << realMutProb << "," << etaC << "," << etaM << ","
            << "," << binVars << "," << binCrossProb << "," << binMutProb << std::endl;


        if (realVars > 0) {
            _os << "\n";

            _os << "# Real Variable Parameters\n";
            _os << "index,initial_value,lower_limit,upper_limit" << std::endl;

            for (int i = 0; i < realVars; i++) {
                _os << i << "," << initialRealVars[i] << "," << realLimits[i].first << "," << realLimits[i].second
                    << std::endl;
            }
        }

        if (binVars > 0) {
            _os << "\n";

            _os << "# Binary Variable Parameters\n";
            _os << "index,lower_limit,upper_limit,num_bits" << std::endl;

            for (int i = 0; i < binVars; i++) {
                _os << i << "," << binLimits[i].first << "," << binLimits[i].second << "," << binBits[i] << std::endl;
            }
        }
        _os.flush();
    }

    void NSGA2::ReportPop(const std::shared_ptr<Population>& _pop, std::ostream& _os) const {
        _os << "\n# Generation " << _pop->generation << "\n";

        _os << "generation"
            << ","
            << "individual"
            << ","
            << "constraint_violation"
            << ","
            << "rank"
            << ","
            << "crowding_dist";

        for (int i = 0; i < objectives; i++) {
            _os << "," << "objective_" << i;
        }

        for (int i = 0; i < constraints; i++) {
            _os << "," << "constraints_" << i;
        }

        for (int i = 0; i < realVars; i++) {
            _os << "," << "real_param_" << i;
        }

        for (int i = 0; i < binVars; i++) {
            for (int j = 0; j < binBits[i]; j++) {
                _os << "," << "binary_param_" << i << "_" << j ;
            }
        }

        _os << std::endl;

        _pop->Report(_os);
        _os.flush();
    }
}  // namespace nsga2
