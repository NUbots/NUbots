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

        bool InitializeFirstGeneration();
        void CompleteGenerationAndAdvance();
        void InitializeNextGeneration();
        bool HasMetOptimisationTerminalCondition();

        // clang-format off
        void SetSeed(const int& _seed) { randGen->SetSeed(_seed); }
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
        void SetInitialPopulationRealVars(const std::vector<std::vector<double>>& _suppliedPopulationRealVars)
                                                { suppliedPopulationRealVars = _suppliedPopulationRealVars;}
        void setSuppliedPop(const bool& _supplied_pop) {supplied_pop = _supplied_pop;}
        // clang-format on

        std::shared_ptr<Population> getCurrentPop();

        bool crowdObj      = true;
        int reportCount    = 0;
        int binMutCount    = 0;
        int realMutCount   = 0;
        int binCrossCount  = 0;
        int realCrossCount = 0;
        int bitLength      = 0;

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
        // Added for an intial population
        bool supplied_pop = false;
        std::vector<std::vector<double>> suppliedPopulationRealVars;

        std::shared_ptr<Population> parentPop      = nullptr;
        std::shared_ptr<Population> childPop       = nullptr;
        std::shared_ptr<Population> combinedPop    = nullptr;
        std::shared_ptr<RandomGenerator<>> randGen = nullptr;

        // Output file streams
        std::ofstream initial_pop_file;
        std::ofstream final_pop_file;
        std::ofstream all_pop_file;
        std::ofstream nsga2_params_file;

        // EarlyStopping
        bool earlyStoppingOneFront      = false;
        bool earlyStoppingNoImprovement = false;

        bool ConfigurationIsValid();
        void CreateStartingPopulations();
        void InitializeReportingStreams();
        void WriteReportHeaders(std::ofstream& _os, std::string file_name) const;
        void ReportParams(std::ofstream& _os, std::string file_name) const;
        void ReportPop(const std::shared_ptr<Population>& pop, std::ofstream& os) const;

        void CompleteGeneration();

        void Selection(const std::shared_ptr<Population>& oldpop, std::shared_ptr<Population>& newpop);
        const Individual& Tournament(const Individual& ind1, const Individual& ind2) const;
        void Crossover(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);
        void SelfAdaptiveSBX(const Individual& parent1,
                             const Individual& parent2,
                             Individual& child1,
                             Individual& child2);
        void Bincross(const Individual& parent1, const Individual& parent2, Individual& child1, Individual& child2);
    };
}  // namespace nsga2

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_NSGA2_HPP
