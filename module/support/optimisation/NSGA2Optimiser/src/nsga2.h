#ifndef NSGA2_H
#define NSGA2_H

#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "population.h"
#include "random.h"

namespace nsga2 {
class NSGA2 {
public:
    NSGA2();
    virtual ~NSGA2();
    int PreEvaluationInitialize();
    void PostEvaluationInitialize();
    void PreEvaluationAdvance();
    void PostEvaluationAdvance();
    // void Evolve();
    void SetSeed(int _seed);
    void SetCrowdObj(bool _crowd);
    void SetRealVariableCount(int _realVars);
    void SetBinVariableCount(int _binVars);
    void SetInitialRealVars(std::vector<double> _initRealVars);
    void SetObjectiveCount(int _objectives);
    void SetContraintCount(int _constraints);
    void SetPopulationSize(int _popSize);
    void SetTargetGenerations(int _generations);
    void SetRealCrossoverProbability(double _realCrossProb);
    void SetBinCrossoverProbability(double _binCrossProb);
    void SetRealMutationProbability(double _realMutProb);
    void SetBinMutationProbability(double _binMutProb);
    void SetEtaC(double _etaC);
    void SetEtaM(double _etaM);
    void SetEpsC(double _epsC);
    void SetBitCount(const std::vector<int> _binBits);
    void SetRealVarLimits(const std::vector<std::pair<double, double>> _realLimits);
    void SetBinVarLimits(const std::vector<std::pair<double, double>> _binLimits);
    void InitStreams();
    void ReportParams(std::ostream& os) const;
    void ReportPop(const Population& pop, std::ostream& os) const;
    void ReportFinalGenerationPop();
    void SetRandomInitialize(bool _randomInitialize);

    void Selection(Population& oldpop, Population& newpop);
    Individual& Tournament(Individual& ind1, Individual& ind2) const;
    void Crossover(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);
    void Realcross(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);
    void Bincross(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);

    Population* parentPop;
    Population* childPop;
    Population* mixedPop;
    RandomGenerator* randGen;

    bool crowdObj;
    int reportCount;
    int binMutCount;
    int realMutCount;
    int binCrossCount;
    int realCrossCount;
    int bitLength;

    std::ofstream fpt1;
    std::ofstream fpt2;
    std::ofstream fpt3;
    std::ofstream fpt4;
    std::ofstream fpt5;

    int currentGen;
    int popSize;
    int generations;

private:
    int realVars;
    int binVars;
    int objectives;
    int constraints;
    double realCrossProb;
    double binCrossProb;
    double realMutProb;
    double binMutProb;
    double etaC;
    double etaM;
    double epsC;
    std::vector<int> binBits;
    std::vector<double> initialRealVars;
    bool randomInitialize;
    std::vector<std::pair<double, double>> realLimits;
    std::vector<std::pair<double, double>> binLimits;
};
}  // namespace nsga2

#endif
