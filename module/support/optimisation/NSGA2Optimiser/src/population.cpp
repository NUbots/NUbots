#include "population.h"

namespace nsga2 {
Population::Population(const int _size,
                       const int _realVars,
                       const int _binVars,
                       const int _constraints,
                       const std::vector<int> _binBits,
                       const std::vector<std::pair<double, double>> _realLimits,
                       const std::vector<std::pair<double, double>> _binLimits,
                       const int _objectives,
                       const double _realMutProb,
                       const double _binMutProb,
                       const double _etaM,
                       const double _epsC,
                       const bool _crowdObj,
                       RandomGenerator* _randGen,
                       const std::vector<double> _initialRealVars) {
    generation = 1;
    crowdObj   = _crowdObj;
    front      = std::vector<std::vector<int>>();

    indConfig.realVars        = _realVars;
    indConfig.realLimits      = _realLimits;
    indConfig.realMutProb     = _realMutProb;
    indConfig.binVars         = _binVars;
    indConfig.binBits         = _binBits;
    indConfig.binLimits       = _binLimits;
    indConfig.binMutProb      = _binMutProb;
    indConfig.objectives      = _objectives;
    indConfig.constraints     = _constraints;
    indConfig.etaM            = _etaM;
    indConfig.epsC            = _epsC;
    indConfig.randGen         = _randGen;
    indConfig.initialRealVars = _initialRealVars;

    size = _size;

    for (int i = 0; i < _size; i++) {
        inds.push_back(Individual(indConfig));
    }
}

Population::~Population() {}

int Population::GetSize() const {
    return inds.size();
}

void Population::Initialize(bool randomInitialize) {
    for (int i = 0; i < size; i++) {
        inds[i].Initialize(i, randomInitialize);
    }
}

void Population::Decode() {
    std::vector<Individual>::iterator it;
    for (it = inds.begin(); it != inds.end(); it++) {
        it->Decode();
    }
}

/*void Population::Evaluate()
{
    std::vector<Individual>::iterator it;
    for (it = inds.begin();
        it != inds.end(); it++)
    {
        it->Evaluate();
    }
}*/

/*void Population::EvaluateInd(int _id)
{
    inds[_id].Evaluate(generation);
}*/
std::vector<double> Population::GetIndReals(int _id) {
    return inds[_id].reals;
}

void Population::SetIndObjectiveScore(int _id, std::vector<double> _objScore) {
    inds[_id].objScore = _objScore;
}

void Population::SetIndConstraints(int _id, std::vector<double> _constraints) {
    inds[_id].constr = _constraints;
}

void Population::CheckConstraints() {
    std::vector<Individual>::iterator it;
    for (it = inds.begin(); it != inds.end(); it++) {
        it->CheckConstraints();
    }
}

void Population::FastNDS() {
    front.resize(1);
    front[0].clear();

    for (int i = 0; i < inds.size(); i++) {
        std::vector<int> dominationList;
        int dominationCount = 0;
        Individual& indP    = inds[i];

        for (int j = 0; j < inds.size(); j++) {
            Individual& indQ = inds[j];

            int comparison = indP.CheckDominance(indQ);
            if (comparison == 1)
                dominationList.push_back(j);
            else if (comparison == -1)
                dominationCount++;
        }

        indP.dominations = dominationCount;
        indP.dominated.clear();
        indP.dominated = dominationList;

        if (indP.dominations == 0) {
            indP.rank = 1;
            front[0].push_back(i);
        }
    }

    std::sort(front[0].begin(), front[0].end());

    int fi = 1;
    while (front[fi - 1].size() > 0) {
        std::vector<int>& fronti = front[fi - 1];
        std::vector<int> Q;
        for (int i = 0; i < fronti.size(); i++) {
            Individual& indP = inds[fronti[i]];

            for (int j = 0; j < indP.dominated.size(); j++) {
                Individual& indQ = inds[indP.dominated[j]];
                indQ.dominations--;

                if (indQ.dominations == 0) {
                    indQ.rank = fi + 1;
                    Q.push_back(indP.dominated[j]);
                }
            }
        }

        fi++;
        front.push_back(Q);
    }
}


void Population::CrowdingDistanceAll() {
    for (int i = 0; i < front.size(); i++) {
        CrowdingDistance(i);
    }
}

void Population::CrowdingDistance(int _frontI) {
    std::vector<int> F = front[_frontI];
    if (F.size() == 0) return;

    const int l = F.size();
    for (int i = 0; i < l; i++) {
        inds[F[i]].crowdDist = 0;
    }
    const int limit = crowdObj ? indConfig.objectives : indConfig.realVars;
    for (int i = 0; i < limit; i++) {
        std::sort(F.begin(), F.end(), comparator_obj(*this, i));

        inds[F[0]].crowdDist = INF;
        if (l > 1) inds[F[l - 1]].crowdDist = INF;

        for (int j = 1; j < l - 1; j++) {
            if (inds[F[j]].crowdDist != INF) {
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
    if (GetSize() < _pop1.GetSize() + _pop2.GetSize())
        std::cout << "Merge: target population not big enough" << std::endl;

    std::copy(_pop1.inds.begin(), _pop1.inds.end(), inds.begin());
    std::copy(_pop2.inds.begin(), _pop2.inds.end(), inds.begin() + _pop1.GetSize());
}

void Population::Report(std::ostream& os) const {
    std::vector<Individual>::const_iterator it;
    for (it = inds.begin(); it != inds.end(); it++) {
        for (int i = 0; i < indConfig.objectives; i++)
            os << it->objScore[i] << "\t";
        for (int i = 0; i < indConfig.constraints; i++)
            os << it->constr[i] << "\t";
        for (int i = 0; i < indConfig.realVars; i++)
            os << it->reals[i] << "\t";
        for (int i = 0; i < indConfig.binVars; i++)
            for (int j = 0; j < indConfig.binBits[i]; j++)
                os << it->gene[i][j] << "\t";

        os << it->constrViolation << "\t" << it->rank << "\t" << it->crowdDist << "\n";
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
