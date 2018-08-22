#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>

#include "nsga2.h"

void fitnessFunction(std::vector<double> &_reals,
	std::vector<double> &_bins, std::vector<std::vector<int>> &_gene,
	std::vector<double> &_objScore, std::vector<double> &_constraints)
{
	_objScore[0] = std::pow(_reals[0], 2.0) + _reals[1] * std::cos(_reals[0]);
	_objScore[1] = std::pow((_reals[0] - 2.0), 2.0) - _reals[1] * _reals[1];
}

int main(void)
{
	nsga2::RandomGenerator randGen;

	int seed = 30;
	randGen.SetSeed(seed);

	int popSize = 20;
	int generations = 10;
	int objectives = 2;
	int constraints = 0;
	int realVars = 2;
	std::vector<std::pair<double,double>> realLimits;
	realLimits.push_back(std::make_pair(0.2, 1.0));
	realLimits.push_back(std::make_pair(0.2, 1.0));
	double realCrossProb = 0.75;
	double realMutProb = 0.6;
	double etaC = 5;
	double etaM = 5;
	int binVars = 0;
	std::vector<int> binBits;
	std::vector<std::pair<double,double>> binLimits;
	double binCrossProb = 0;
	double binMutProb   = 0;

	std::cout << "Starting..." << std::endl;

	nsga2::NSGA2 nsga2;
	nsga2.randGen = &randGen;
	nsga2.SetRealVariableCount(realVars);
	nsga2.SetBinVariableCount(binVars);
	nsga2.SetObjectiveCount(objectives);
	nsga2.SetContraintCount(constraints);
	nsga2.SetPopulationSize(popSize);
	nsga2.SetTargetGenerations(generations);
	nsga2.SetRealCrossoverProbability(realCrossProb);
	nsga2.SetBinCrossoverProbability(binCrossProb);
	nsga2.SetRealMutationProbability(realMutProb);
	nsga2.SetBinMutationProbability(binMutProb);
	nsga2.SetEtaC(etaC);
	nsga2.SetEtaM(etaM);
	nsga2.SetBitCount(binBits);
	nsga2.SetRealVarLimits(realLimits);
	nsga2.SetBinVarLimits(binLimits);
	nsga2.SetFitnessFunction(fitnessFunction);

	if (nsga2.Initialize() != 0)
		return 1;

	nsga2.Evolve();
	
	return 0;
}