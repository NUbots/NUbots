#include "Population.hpp"

#include <nuclear>

namespace nsga2 {
    Population::Population(const int& _size,
                           const int& _realVars,
                           const int& _binVars,
                           const int& _constraints,
                           const std::vector<int>& _binBits,
                           const std::vector<std::pair<double, double>>& _realLimits,
                           const std::vector<std::pair<double, double>>& _binLimits,
                           const int& _objectives,
                           const double& _realMutProb,
                           const double& _binMutProb,
                           const double& _etaM,
                           const double& _epsC,
                           const bool& _crowdObj,
                           std::shared_ptr<RandomGenerator<>> _randGen,
                           const std::vector<double>& _initialRealVars)
        : indConfig({_realVars,
                     _realLimits,
                     _realMutProb,
                     _binVars,
                     _binBits,
                     _binLimits,
                     _binMutProb,
                     _objectives,
                     _constraints,
                     _etaM,
                     _epsC,
                     _randGen,
                     _initialRealVars})
        , size(_size)
        , crowdObj(_crowdObj) {

        for (int i = 0; i < _size; i++) {
            inds.emplace_back(indConfig);
        }
    }

    void Population::Initialize(const bool& randomInitialize) {
        for (int i = 0; i < size; i++) {
            inds[i].Initialize(i, randomInitialize);
        }
    }
    void Population::Decode() {
        for (auto& ind : inds) {
            ind.Decode();
        }
    }

    void Population::SetGeneration(const int _generation) {
        generation = _generation;
        for (auto& ind : inds) {
            ind.generation = _generation;
        }
    }

    int Population::GetGeneration() const {
        return generation;
    }

    void Population::SetIds() {
        for (std::size_t i = 0; i < inds.size(); i++) {
            inds[i].id = i;
        }
    }

    bool Population::IsReadyToEvalulate() const {
        if(lockedByGeneticAlgorithm) {
            return false;
        }
        for (auto& ind : inds) {
            if(ind.id == -1) {
                return false;
            }
            if(ind.generation == -1) {
                return false;
            }
        }
        return true;
    }

    void Population::resetEvaluationState() {
        currentInd = 0;
    }

    std::optional<Individual> Population::GetNextIndividual() {
        if(!IsReadyToEvalulate() || currentInd >= inds.size()) {
            return std::nullopt;
        } else {
            return std::optional<Individual>{inds[currentInd++]};
        }
    }

    bool Population::AreAllEvaluated() const {
        for (auto& ind : inds) {
            if(!ind.evaluated) {
                return false;
            }
        }
        return true;
    }

    void Population::SetEvaluationResults(const int& _id, const std::vector<double>& _objScore, const std::vector<double>& _constraints) {
        inds[_id].objScore = _objScore;
        inds[_id].constr = _constraints;
        inds[_id].CheckConstraints();
    }

    //Fast Non-Dominated Sort. This calculates the fronts in the population.
    void Population::FastNDS() {
        //Reset Front
        fronts.resize(1);
        fronts[0].clear();

        //Compare each individual `p` to each other individual `q` (also compares p to itself, but doesn't matter)
        for (std::size_t p = 0; p < inds.size(); p++) {
            auto& indP = inds[p];
            indP.dominationList.clear(); //Reset the set of individuals that P dominates
            indP.dominatedByCounter = 0; //Reset the count of individuals that dominate P

            for (std::size_t q = 0; q < inds.size(); q++) {
                const auto& indQ = inds[q];

                int comparison = indP.CheckDominance(indQ);
                if (comparison == 1) {
                    //If P dominates Q, Add Q to the solutions that P dominates
                    indP.dominationList.push_back(q);
                } else if (comparison == -1) {
                    //If Q dominates P, Increment the number of individuals that dominate P
                    indP.dominatedByCounter++;
                }
            }

            if (indP.dominatedByCounter == 0) {
                //If no other individuals dominate P, then P must be in the first Front (i.e. Rank 1)
                indP.rank = 1;
                fronts[0].push_back(p);
            }
        }

        //Sort the first front (sorting by index, since front is `std::vector<std::vector<int>>`)
        std::sort(fronts[0].begin(), fronts[0].end());

        std::size_t front_index = 1;
        while (fronts[front_index - 1].size() > 0) { //While we haven't encountered an empty front
            std::vector<int>& fronti = fronts[front_index - 1];
            std::vector<int> next_front; //Known as Q in the original paper, this holds members of the next front
            for (std::size_t p = 0; p < fronti.size(); p++) {
                Individual& indP = inds[fronti[p]];

                //For each of the individuals in the domination list, check if it's part of the next front
                for (std::size_t q = 0; q < indP.dominationList.size(); q++) {
                    auto& indQ = inds[indP.dominationList[q]];
                    indQ.dominatedByCounter--; //Reduce the counter, as we are no longer considering P vs this Q

                    if (indQ.dominatedByCounter == 0) {
                        //If no other individuals outside of current front dominate Q, then Q must be in the next Front
                        indQ.rank = front_index + 1;
                        next_front.push_back(indP.dominationList[q]);
                    }
                }
            }
            fronts.push_back(next_front);
            front_index++;
        }
    }

    void Population::CrowdingDistanceAll() {
        for (int i = 0; i < int(fronts.size()); i++) {
            CrowdingDistance(i);
        }
    }

    void Population::CrowdingDistance(const int& _frontI) {
        std::vector<int>& F = fronts[_frontI];
        if (F.size() == 0) {
            return;
        }

        const int l = F.size();
        for (int i = 0; i < l; i++) {
            inds[F[i]].crowdDist = 0;
        }
        const int limit = crowdObj ? indConfig.objectives : indConfig.realVars;
        for (int i = 0; i < limit; i++) {
            std::sort(F.begin(), F.end(), [&](const int& a, const int& b) {
                return crowdObj ? inds[a].objScore[i] < inds[b].objScore[i] : inds[a].reals[i] < inds[b].reals[i];
            });

            inds[F[0]].crowdDist = std::numeric_limits<double>::infinity();
            if (l > 1)
                inds[F[l - 1]].crowdDist = std::numeric_limits<double>::infinity();

            for (int j = 1; j < l - 1; j++) {
                if (inds[F[j]].crowdDist != std::numeric_limits<double>::infinity()) {
                    if (crowdObj && inds[F[l - 1]].objScore[i] != inds[F[0]].objScore[i]) {
                        inds[F[j]].crowdDist += (inds[F[j + 1]].objScore[i] - inds[F[j - 1]].objScore[i])
                                                / (inds[F[l - 1]].objScore[i] - inds[F[0]].objScore[i]);
                    }
                    else if (!crowdObj && inds[F[l - 1]].reals[i] != inds[F[0]].reals[i]) {
                        inds[F[j]].crowdDist += (inds[F[j + 1]].reals[i] - inds[F[j - 1]].reals[i])
                                                / (inds[F[l - 1]].reals[i] - inds[F[0]].reals[i]);
                    }
                }
            }
        }
    }

    void Population::Merge(const Population& _pop1, const Population& _pop2) {
        if (GetSize() < _pop1.GetSize() + _pop2.GetSize()) {
            NUClear::log<NUClear::WARN>("Merge: target population not big enough");
            inds.reserve(_pop1.GetSize() + _pop2.GetSize());
        }

        std::copy(_pop1.inds.begin(), _pop1.inds.end(), inds.begin());
        std::copy(_pop2.inds.begin(), _pop2.inds.end(), inds.begin() + _pop1.GetSize());
    }

    void Population::Report(std::ostream& os) const {
        std::vector<Individual>::const_iterator it;

        for (it = inds.begin(); it != inds.end(); it++) {
            os << generation << "," << it->id << ",";

            for (int i = 0; i < indConfig.objectives; i++) {
                os << it->objScore[i] << ",";
            }

            for (int i = 0; i < indConfig.constraints; i++) {
                os << it->constr[i] << ",";
            }

            for (int i = 0; i < indConfig.realVars; i++) {
                os << it->reals[i] << ",";
            }

            for (int i = 0; i < indConfig.binVars; i++) {
                for (int j = 0; j < indConfig.binBits[i]; j++) {
                    os << it->gene[i][j] << ",";
                }
            }

            os << it->constrViolation << "," << it->rank << "," << it->crowdDist << std::endl;
        }
    }

    std::pair<int, int> Population::Mutate() {
        std::pair<int, int> mutCount    = std::make_pair(0, 0);
        std::pair<int, int> indMutCount = std::make_pair(0, 0);
        std::vector<Individual>::iterator it;
        for (it = inds.begin(); it != inds.end(); it++) {
            indMutCount = it->Mutate();
            mutCount.first += indMutCount.first;
            mutCount.second += indMutCount.second;
        }
        return mutCount;
    }

    std::ostream& operator<<(std::ostream& _os, const Population& _pop) {
        _os << "Population: {\n";
        std::vector<Individual>::const_iterator it;
        for (it = _pop.inds.begin(); it != _pop.inds.end(); it++) {
            _os << *it;
        }
        _os << "}";
        return _os;
    }
}  // namespace nsga2
