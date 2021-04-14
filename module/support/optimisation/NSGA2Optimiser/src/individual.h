#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H

#include <iostream>
#include <ostream>
#include <vector>

#include "random.h"

namespace nsga2 {
struct IndividualConfigurator {
    int realVars;
    std::vector<std::pair<double, double>> realLimits;
    double realMutProb;
    int binVars;
    std::vector<int> binBits;
    std::vector<std::pair<double, double>> binLimits;
    double binMutProb;
    int objectives;
    int constraints;
    double etaM;
    double epsC;
    RandomGenerator* randGen;
    std::vector<double> initialRealVars;
};

class Individual {
public:
    Individual(const IndividualConfigurator& _config);
    virtual ~Individual();
    void Initialize(int _id, bool randomInitialize);
    void Decode();
    void Evaluate(int _generation);
    std::pair<int, int> Mutate();
    int CheckDominance(const Individual& _b) const;
    void CheckConstraints();

    int id;
    int generation;
    int rank;
    double constrViolation;
    std::vector<double> reals;
    std::vector<std::vector<int>> gene;
    std::vector<double> bins;
    std::vector<double> objScore;
    std::vector<double> constr;
    double crowdDist;

    int dominations;
    std::vector<int> dominated;
    bool evaluated;

private:
    int realMutate();
    int binMutate();
    const IndividualConfigurator* config;
};
std::ostream& operator<<(std::ostream& _os, const Individual& _ind);
}  // namespace nsga2

#endif
