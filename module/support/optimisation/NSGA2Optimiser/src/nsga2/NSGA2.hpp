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
        NSGA2() : rand_gen(std::make_shared<RandomGenerator<>>()) {}

        bool InitializeFirstGeneration();
        void CompleteGenerationAndAdvance();
        void InitializeNextGeneration();
        bool HasMetOptimisationTerminalCondition();

        // clang-format off
        void SetSeed(const int& _seed) { rand_gen->SetSeed(_seed); }
        void SetRealVariableCount(const int& real_vars_) { real_vars = real_vars_; }
        void SetBinVariableCount(const int& bin_vars_) { bin_vars = bin_vars_; }
        void SetObjectiveCount(const int& objectives_) { objectives = objectives_; }
        void SetContraintCount(const int& constraints_) { constraints = constraints_; }
        void SetPopulationSize(const int& pop_size_) { pop_size = pop_size_; }
        void SetTargetGenerations(const int& _generations) { generations = _generations; }
        void SetRealCrossoverProbability(const double& real_cross_prob_) { real_cross_prob = real_cross_prob_; }
        void SetBinCrossoverProbability(const double& bin_cross_prob_) { bin_cross_prob = bin_cross_prob_; }
        void SetRealMutationProbability(const double& real_mut_prob_) { real_mut_prob = real_mut_prob_; }
        void SetBinMutationProbability(const double& bin_mut_prob_) { bin_mut_prob = bin_mut_prob_; }
        void SetEtaC(const double& etaC_) { etaC = etaC_; }
        void SetEtaM(const double& etaM_) { etaM = etaM_; }
        void SetEpsC(const double& epsC_) { epsC = epsC_; }
        void SetBitCount(const std::vector<int>& bin_bits_) { bin_bits = bin_bits_; }
        void SetRealVarLimits(const std::vector<std::pair<double, double>>& real_limits_) { real_limits = real_limits_; }
        void SetBinVarLimits(const std::vector<std::pair<double, double>>& bin_limits_) { bin_limits = bin_limits_; }
        void SetInitialRealVars(const std::vector<double>& init_real_vars_) { initial_real_vars = init_real_vars_; }
        void SetInitialPopulationRealVars(const std::vector<std::vector<double>>& supplied_population_real_vars_)
                                                { supplied_population_real_vars = supplied_population_real_vars_;}
        void setSuppliedPop(const bool& supplied_pop_) {supplied_pop = supplied_pop_;}
        // clang-format on

        std::shared_ptr<Population> getCurrentPop();

        bool crowd_obj       = true;
        int report_count     = 0;
        int bin_mut_count    = 0;
        int real_mut_count   = 0;
        int bin_cross_count  = 0;
        int real_cross_count = 0;
        int bit_length       = 0;

        int current_gen;
        int pop_size;
        int generations;

    private:
        int real_vars            = -1;
        int bin_vars             = -1;
        int objectives           = -1;
        int constraints          = -1;
        double real_cross_prob   = -1.0;
        double bin_cross_prob    = -1.0;
        double real_mut_prob     = -1.0;
        double bin_mut_prob      = -1.0;
        double etaC              = -1.0;
        double etaM              = -1.0;
        double epsC              = std::numeric_limits<double>::epsilon();
        std::vector<int> bin_bits = {};
        std::vector<double> initial_real_vars;
        bool random_initialize                             = {};
        std::vector<std::pair<double, double>> real_limits = {};
        std::vector<std::pair<double, double>> bin_limits  = {};
        // Added for an intial population
        bool supplied_pop = false;
        std::vector<std::vector<double>> supplied_population_real_vars;

        std::shared_ptr<Population> parent_pop      = nullptr;
        std::shared_ptr<Population> child_pop       = nullptr;
        std::shared_ptr<Population> combined_pop    = nullptr;
        std::shared_ptr<RandomGenerator<>> rand_gen = nullptr;

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
