#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_NSGA2_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_NSGA2_HPP

#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "Population.hpp"
#include "Random.hpp"

namespace nsga2 {
    class NSGA2 {
    public:
        NSGA2() : randGen(std::make_shared<RandomGenerator<>>()) {}
        virtual ~NSGA2() = default;
        int PreEvaluationInitialize();
        void PostEvaluationInitialize();
        void PreEvaluationAdvance();
        void PostEvaluationAdvance();

        // clang-format off
        void SetSeed(const int& _seed) { randGen->SetSeed(_seed); }
        void SetCrowdObj(const bool& _crowd) { crowdObj = _crowd; }
        void SetRealVariableCount(const int& _realVars) { realVars = _realVars; }
        void SetBinVariableCount(const int& _binVars) { binVars = _binVars; }
        void SetObjectiveCount(const int& _objectives) { objectives = _objectives; }
        void SetContraintCount(const int& _constraints) { constraints = _constraints; }
        void SetPopulationSize(const int& _popSize) { popSize = _popSize; }
        void SetTargetGenerations(const int& _generations) { generations = _generations; }
        void SetRealCrossoverProbability(const double& _realCrossProb) { realCrossProb = _realCrossProb; }
        void SetBinCrossoverProbability(const double& _binCrossProb) { binCrossProb = _binCrossProb; }
        void SetRealMutationProbability(const double& _realMutProb) { realMutProb = _realMutProb; }
        void SetBinMutationProbability(const double& _binMutProb) { binMutProb = _binMutProb; }
        void SetEtaC(const double& _etaC) { etaC = _etaC; }
        void SetEtaM(const double& _etaM) { etaM = _etaM; }
        void SetEpsC(const double& _epsC) { epsC = _epsC; }
        void SetBitCount(const std::vector<int>& _binBits) { binBits = _binBits; }
        void SetRealVarLimits(const std::vector<std::pair<double, double>>& _realLimits) { realLimits = _realLimits; }
        void SetBinVarLimits(const std::vector<std::pair<double, double>>& _binLimits) { binLimits = _binLimits; }
        void SetInitialRealVars(const std::vector<double>& _initRealVars) { initialRealVars = _initRealVars; }
        void SetRandomInitialize(const bool& _randomInitialize) { randomInitialize = _randomInitialize; }
        // clang-format on

        void InitStreams();
        void ReportParams(std::ostream& os) const;
        void ReportPop(const std::shared_ptr<Population>& pop, std::ostream& os) const;
        void ReportFinalGenerationPop();

        void Selection(const std::shared_ptr<Population>& oldpop, std::shared_ptr<Population>& newpop);
        const Individual& Tournament(const Individual& ind1, const Individual& ind2) const;
        void Crossover(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);
        void Realcross(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);
        void Bincross(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);

        std::shared_ptr<Population> parentPop      = nullptr;
        std::shared_ptr<Population> childPop       = nullptr;
        std::shared_ptr<Population> mixedPop       = nullptr;
        std::shared_ptr<RandomGenerator<>> randGen = nullptr;

        bool crowdObj      = true;
        int reportCount    = 0;
        int binMutCount    = 0;
        int realMutCount   = 0;
        int binCrossCount  = 0;
        int realCrossCount = 0;
        int bitLength      = 0;

        std::ofstream fpt1;
        std::ofstream fpt2;
        std::ofstream fpt3;
        std::ofstream fpt4;
        std::ofstream fpt5;

        int currentGen;
        int popSize;
        int generations;

    private:
        int realVars             = -1;
        int binVars              = -1;
        int objectives           = -1;
        int constraints          = -1;
        double realCrossProb     = -1.0;
        double binCrossProb      = -1.0;
        double realMutProb       = -1.0;
        double binMutProb        = -1.0;
        double etaC              = -1.0;
        double etaM              = -1.0;
        double epsC              = std::numeric_limits<double>::epsilon();
        std::vector<int> binBits = {};
        std::vector<double> initialRealVars;
        bool randomInitialize                             = {};
        std::vector<std::pair<double, double>> realLimits = {};
        std::vector<std::pair<double, double>> binLimits  = {};
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_NSGA2_HPP
