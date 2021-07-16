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
        , size(_size) {

        for (int i = 0; i < _size; i++) {
            inds.emplace_back(indConfig);
        }
    }

    void Population::Initialize() {
        for (int i = 0; i < size; i++) {
            inds[i].Initialize(i);
        }
    }
    void Population::Decode() {
        for (auto& ind : inds) {
            ind.Decode();
        }
    }

    void Population::SetIndividualsGeneration(const int _generation) {
        for (auto& ind : inds) {
            ind.generation = _generation;
        }
    }

    void Population::SetIds() {
        for (std::size_t i = 0; i < inds.size(); i++) {
            inds[i].id = i;
        }
    }

    void Population::resetCurrentIndividualIndex() {
        currentInd = 0;
    }

    std::optional<Individual> Population::GetNextIndividual() {
        if(!initialised || currentInd >= inds.size()) {
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
        for (std::size_t i = 0; i < fronts.size(); i++) {
            CrowdingDistance(i);
        }
    }

    // Calculate how close the next nearest solution is. Boundary solutions have infinite distance.
    // This allows us to prioritise boundary solutions over solutions crowded together.
    void Population::CrowdingDistance(const int& _frontIndex) {
        std::vector<int>& F = fronts[_frontIndex];
        const std::size_t front_size = F.size();
        if (front_size == 0) {
            return; // Don't do anything with an empty front
        }

        for (std::size_t i = 0; i < front_size; i++) {
            inds[F[i]].crowdDist = 0; //Initialise crowding distance
        }

        for (int i = 0; i < indConfig.objectives; i++) { //For each objective
            //Sort the front by objective value
            std::sort(F.begin(), F.end(), [&](const int& a, const int& b) {
                return inds[a].objScore[i] < inds[b].objScore[i];
            });

            //Give the bondary solutions infinite distance
            inds[F[0]].crowdDist = std::numeric_limits<double>::infinity();
            inds[F[front_size - 1]].crowdDist = std::numeric_limits<double>::infinity();

            // Calculate the crowding distance of non-boundary solutions
            for (std::size_t j = 1; j < front_size - 1; j++) {
                if (inds[F[j]].crowdDist != std::numeric_limits<double>::infinity()) {
                    if (inds[F[front_size - 1]].objScore[i] != inds[F[0]].objScore[i]) {
                        inds[F[j]].crowdDist += (inds[F[j + 1]].objScore[i] - inds[F[j - 1]].objScore[i])
                                                / (inds[F[front_size - 1]].objScore[i] - inds[F[0]].objScore[i]);
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

    void Population::Report(std::ostream& os, int currentGen) const {
        for (auto it = inds.begin(); it != inds.end(); it++) {
            it->Report(os, currentGen);
        }
    }
}  // namespace nsga2
