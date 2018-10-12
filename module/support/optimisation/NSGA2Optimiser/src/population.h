#ifndef POPULATION_H
#define POPULATION_H

#include <algorithm>

#include "individual.h"

#define EPS 1e-14
#define INF 1e+14

namespace nsga2 {
class Population {
public:
    Population(const int _size,
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
               const std::vector<double> _initialRealVars);
    virtual ~Population();
    void Initialize(bool randomInitialize);
    void Decode();
    // void Evaluate();
    void EvaluateInd(int _id);
    std::vector<double> GetIndReals(int _id);
    void SetIndObjectiveScore(int _id, std::vector<double> _objScore);
    void SetIndConstraints(int _id, std::vector<double> _constraints);
    void CheckConstraints();
    void FastNDS();
    void CrowdingDistanceAll();
    void CrowdingDistance(int _frontI);
    std::pair<int, int> Mutate();
    void Merge(const Population& _pop1, const Population& _pop2);
    void Report(std::ostream& _os) const;
    int GetSize() const;

    std::vector<Individual> inds;
    std::vector<std::vector<int>> front;

    int size;
    bool crowdObj;
    int generation;

private:
    IndividualConfigurator indConfig;
};
std::ostream& operator<<(std::ostream& _os, const Population& _pop);

struct comparator_obj {
    comparator_obj(const Population& population, int index) : pop(population), m(index){};
    const Population& pop;
    int m;
    bool operator()(int i, int j) {
        return pop.crowdObj ? pop.inds[i].objScore[m] < pop.inds[j].objScore[m]
                            : pop.inds[i].reals[m] < pop.inds[j].reals[m];
    };
};
}  // namespace nsga2

#endif
