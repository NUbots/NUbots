#include "NSGA2Optimiser.h"

#include "extension/Configuration.h"

#include "message/support/optimisation/NSGA2EvaluationRequest.h"
#include "message/support/optimisation/NSGA2FitnessScores.h"

namespace module {
namespace support {
namespace optimisation {

    using extension::Configuration;
    using message::support::optimisation::NSGA2FitnessScores;
    using message::support::optimisation::NSGA2EvaluationRequest;


    NSGA2Optimiser::NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NSGA2Optimiser.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file NSGA2Optimiser.yaml

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

            nsga2Algorithm.randGen = &randGen;
            nsga2Algorithm.SetRealVariableCount(realVars);
            nsga2Algorithm.SetBinVariableCount(binVars);
            nsga2Algorithm.SetObjectiveCount(objectives);
            nsga2Algorithm.SetContraintCount(constraints);
            nsga2Algorithm.SetPopulationSize(popSize);
            nsga2Algorithm.SetTargetGenerations(generations);
            nsga2Algorithm.SetRealCrossoverProbability(realCrossProb);
            nsga2Algorithm.SetBinCrossoverProbability(binCrossProb);
            nsga2Algorithm.SetRealMutationProbability(realMutProb);
            nsga2Algorithm.SetBinMutationProbability(binMutProb);
            nsga2Algorithm.SetEtaC(etaC);
            nsga2Algorithm.SetEtaM(etaM);
            nsga2Algorithm.SetBitCount(binBits);
            nsga2Algorithm.SetRealVarLimits(realLimits);
            nsga2Algorithm.SetBinVarLimits(binLimits);

            if (nsga2Algorithm.PreEvaluationInitialize() != 0)
                return 1;

            // This starts the algorithm
            std::cout << "Evaluating gen 0 ind 0" << std::endl;
            requestIndEvaluation(0,
                nsga2Algorithm.parentPop->generation,
                nsga2Algorithm.parentPop->GetIndReals(0));
            //nsga2Algorithm.Evolve();
        });

        on<Trigger<NSGA2FitnessScores>>().then(
            [this](const NSGA2FitnessScores& scores)
            {
                // Set the objScore and constraints

                // move on to next individual
                if (scores.generation == 0) // INITIAL GENERATION
                {
                    nsga2Algorithm.parentPop->
                        SetIndObjectiveScore(scores.id, scores.objScore);
                    nsga2Algorithm.parentPop->
                        SetIndConstraints(scores.id, scores.constraints);

                    if (scores.id < nsga2Algorithm.popSize - 1)
                    {
                        std::cout << "Evaluating gen 0 ind " << scores.id + 1 << std::endl;
                        requestIndEvaluation(scores.id + 1,
                            nsga2Algorithm.parentPop->generation,
                            nsga2Algorithm.parentPop->GetIndReals(scores.id + 1));
                    }
                    else if (scores.id == nsga2Algorithm.popSize - 1)
                    {
                        nsga2Algorithm.PostEvaluationInitialize();
                        nsga2Algorithm.PreEvaluationAdvance();

                        std::cout << "Evaluating gen 1 ind 0" << std::endl;
                        requestIndEvaluation(0,
                            nsga2Algorithm.childPop->generation,
                            nsga2Algorithm.childPop->GetIndReals(0));
                        //nsga2Algorithm.childPop->EvaluateInd(0);
                    }
                }
                else // FOLLOWING GENERATIONS
                {
                    nsga2Algorithm.childPop->
                        SetIndObjectiveScore(scores.id, scores.objScore);
                    nsga2Algorithm.childPop->
                        SetIndConstraints(scores.id, scores.constraints);

                    if (scores.id < nsga2Algorithm.popSize - 1)
                    {
                        std::cout << "Evaluating gen " << scores.generation << " ind " << scores.id + 1 << std::endl;
                        requestIndEvaluation(scores.id + 1,
                            nsga2Algorithm.childPop->generation,
                            nsga2Algorithm.childPop->GetIndReals(scores.id + 1));
                        //nsga2Algorithm.childPop->EvaluateInd(scores.id + 1);
                    }
                    else if (scores.id == nsga2Algorithm.popSize - 1)
                    {
                        if (scores.generation == nsga2Algorithm.generations) // FINAL GENERATION
                        {
                            nsga2Algorithm.PostEvaluationAdvance();
                            nsga2Algorithm.ReportFinalGenerationPop();
                        }
                        else
                        {
                            nsga2Algorithm.PostEvaluationAdvance();
                            nsga2Algorithm.PreEvaluationAdvance();

                            std::cout << "Evaluating gen " << scores.generation + 1 << " ind 0" << std::endl;
                            requestIndEvaluation(scores.id + 1,
                                nsga2Algorithm.childPop->generation,
                                nsga2Algorithm.childPop->GetIndReals(scores.id + 1));
                        }
                    }
                }
        });
    }

    void NSGA2Optimiser::requestIndEvaluation(int _id, int _generation, const std::vector<double>& _reals)
    {
        std::unique_ptr<NSGA2EvaluationRequest> request = std::make_unique<NSGA2EvaluationRequest>();
        request->id = _id;
        request->generation = _generation;
        request->reals = _reals;
        emit(request); // send to the kick engine all the _reals for the population
    }
}
}
}
