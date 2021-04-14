#include "NSGA2Optimiser.h"

#include "extension/Configuration.h"

#include "message/support/optimisation/NSGA2EvaluationRequest.h"
#include "message/support/optimisation/NSGA2FitnessScores.h"
#include "message/support/optimisation/NSGA2Terminate.h"
#include "utility/file/fileutil.h"
#include "utility/input/ServoID.h"

namespace module {
namespace support {
    namespace optimisation {

        using extension::Configuration;
        using extension::Script;
        using message::support::optimisation::NSGA2EvaluationRequest;
        using message::support::optimisation::NSGA2FitnessScores;
        using message::support::optimisation::NSGA2Terminate;
        using ServoID = utility::input::ServoID;


        NSGA2Optimiser::NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

            on<Configuration>("NSGA2Optimiser.yaml").then([this](const Configuration& config) {
                // Use configuration here from file NSGA2Optimiser.yaml

                int seed = 666;
                randGen.SetSeed(seed);

                int popSize     = 40;
                int generations = 300;
                int objectives  = 2;
                int constraints = 2;
                int realVars    = 2;
                std::vector<std::pair<double, double>> realLimits;
                // realLimits.push_back(std::make_pair(0.01, 0.3));
                // realLimits.push_back(std::make_pair(45 * 3.14 / 180, 100 * 3.14 / 180));

                std::vector<double> initialValues;
                bool optimiseScript = true;
                if (optimiseScript) {
                    ::extension::Script script =
                        YAML::LoadFile("scripts/nubotsvm/KickRightArmsExtended.yaml").as<Script>();

                    realVars     = 0;
                    double delta = 0.5;
                    for (int i = 1; i < script.frames.size() - 1; i++) {
                        for (auto& target : script.frames[i].targets) {
                            initialValues.push_back((double) target.position);

                            if (target.id == ServoID::L_KNEE || target.id == ServoID::L_ANKLE_PITCH
                                || target.id == ServoID::L_ANKLE_ROLL) {
                                realLimits.push_back(std::make_pair((double) target.position - (delta * 0.75),
                                                                    (double) target.position + (delta * 0.75)));
                            }
                            else if (target.id == ServoID::L_SHOULDER_PITCH || target.id == ServoID::R_SHOULDER_PITCH) {
                                realLimits.push_back(std::make_pair((double) target.position - (delta * 2.5),
                                                                    (double) target.position + (delta * 2.5)));
                            }
                            else if (target.id == ServoID::L_SHOULDER_ROLL) {
                                realLimits.push_back(std::make_pair((double) target.position - (delta * 2.5),
                                                                    (double) target.position + (delta * 0.125)));
                            }
                            else if (target.id == ServoID::R_SHOULDER_ROLL) {
                                realLimits.push_back(std::make_pair((double) target.position - (delta * 0.125),
                                                                    (double) target.position + (delta * 2.5)));
                            }
                            else if (target.id == ServoID::L_ELBOW || target.id == ServoID::R_ELBOW) {
                                realLimits.push_back(std::make_pair((double) target.position - (delta * 2.5),
                                                                    (double) target.position + (delta * 2.5)));
                            }
                            else {
                                realLimits.push_back(
                                    std::make_pair((double) target.position - delta, (double) target.position + delta));
                            }
                            /*if (target.id == ServoID::R_SHOULDER_PITCH) {
                                realLimits.push_back(std::make_pair((double)target.position - delta,
                            (double)target.position + delta));
                            }
                            else if (target.id == ServoID::L_SHOULDER_PITCH) {
                                realLimits.push_back(std::make_pair((double)target.position - delta,
                            (double)target.position + delta));
                            }
                            else if (target.id == ServoID::R_SHOULDER_ROLL) {
                                realLimits.push_back(std::make_pair(-2.0, 0.0));
                            }
                            else if (target.id == ServoID::L_SHOULDER_ROLL) {
                                realLimits.push_back(std::make_pair(0.0, 2.0));
                            }
                            else if (target.id == ServoID::R_ELBOW) {
                                realLimits.push_back(std::make_pair(-2.75, 0.0));
                            }
                            else if (target.id == ServoID::L_ELBOW) {
                                realLimits.push_back(std::make_pair(-2.75, 0.0));
                            }
                            else if (target.id == ServoID::R_HIP_YAW) {
                                realLimits.push_back(std::make_pair(-0.1, 0.1));
                            }
                            else if (target.id == ServoID::L_HIP_YAW) {
                                realLimits.push_back(std::make_pair(-0.1, 0.1));
                            }
                            else if (target.id == ServoID::R_HIP_ROLL) {
                                realLimits.push_back(std::make_pair(-0.1, 0.1));
                            }
                            else if (target.id == ServoID::L_HIP_ROLL) {
                                realLimits.push_back(std::make_pair(-0.1, 0.1));
                            }
                            else if (target.id == ServoID::R_HIP_PITCH) {

                                realLimits.push_back(std::make_pair(-1.0, 0.5));
                            }
                            else if (target.id == ServoID::L_HIP_PITCH) {
                                realLimits.push_back(std::make_pair(-1.0, 0.5));

                            }
                            else if (target.id == ServoID::R_KNEE) {
                                realLimits.push_back(std::make_pair(0.0, 2.75));
                            }
                            else if (target.id == ServoID::L_KNEE) {
                                realLimits.push_back(std::make_pair(0.0, 2.75));

                            }
                            else if (target.id == ServoID::R_ANKLE_PITCH) {
                                realLimits.push_back(std::make_pair(-1.0, 1.0));
                            }
                            else if (target.id == ServoID::L_ANKLE_PITCH) {
                                realLimits.push_back(std::make_pair(-1.0, 1.0));

                            }
                            else if (target.id == ServoID::R_ANKLE_ROLL) {
                                realLimits.push_back(std::make_pair(-1.0, 1.0));
                            }
                            else if (target.id == ServoID::L_ANKLE_ROLL) {
                                realLimits.push_back(std::make_pair(-1.0, 1.0));

                            }
                            else if (target.id == ServoID::HEAD_YAW){
                                realLimits.push_back(std::make_pair(-1.5, 1.5));
                            }
                            else if (target.id == ServoID::HEAD_PITCH){
                                realLimits.push_back(std::make_pair(-1.0, 1.0));
                            }*/

                            realVars++;
                        }
                    }
                }
                double realCrossProb = 0.8;
                double realMutProb   = 0.025;
                double etaC          = 12;
                double etaM          = 30;
                int binVars          = 0;
                std::vector<int> binBits;
                std::vector<std::pair<double, double>> binLimits;
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
                nsga2Algorithm.SetRandomInitialize(false);
                nsga2Algorithm.SetInitialRealVars(initialValues);

                if (nsga2Algorithm.PreEvaluationInitialize() == 0) {
                    // This starts the algorithm
                    std::cout << "Evaluating gen " << nsga2Algorithm.parentPop->generation << " ind 0" << std::endl;
                    requestIndEvaluation(
                        0, nsga2Algorithm.parentPop->generation, nsga2Algorithm.parentPop->GetIndReals(0));
                    // nsga2Algorithm.Evolve();
                }
            });

            on<Trigger<NSGA2FitnessScores>, Single>().then([this](const NSGA2FitnessScores& scores) {
                // Set the objScore and constraints

                // move on to next individual
                if (scores.generation == 1)  // INITIAL GENERATION
                {
                    nsga2Algorithm.parentPop->SetIndObjectiveScore(scores.id, scores.objScore);
                    nsga2Algorithm.parentPop->SetIndConstraints(scores.id, scores.constraints);

                    if (scores.id < nsga2Algorithm.popSize - 1) {
                        std::cout << "Evaluating gen " << nsga2Algorithm.parentPop->generation << " ind "
                                  << scores.id + 1 << std::endl;
                        requestIndEvaluation(scores.id + 1,
                                             nsga2Algorithm.parentPop->generation,
                                             nsga2Algorithm.parentPop->GetIndReals(scores.id + 1));
                    }
                    else if (scores.id == nsga2Algorithm.popSize - 1) {
                        nsga2Algorithm.PostEvaluationInitialize();
                        nsga2Algorithm.PreEvaluationAdvance();

                        std::cout << "Evaluating gen " << nsga2Algorithm.childPop->generation << " ind 0" << std::endl;
                        requestIndEvaluation(
                            0, nsga2Algorithm.childPop->generation, nsga2Algorithm.childPop->GetIndReals(0));
                        // nsga2Algorithm.childPop->EvaluateInd(0);
                    }
                }
                else  // FOLLOWING GENERATIONS
                {
                    nsga2Algorithm.childPop->SetIndObjectiveScore(scores.id, scores.objScore);
                    nsga2Algorithm.childPop->SetIndConstraints(scores.id, scores.constraints);

                    if (scores.id < nsga2Algorithm.popSize - 1) {
                        std::cout << "Evaluating gen " << nsga2Algorithm.childPop->generation << " ind "
                                  << scores.id + 1 << std::endl;
                        requestIndEvaluation(scores.id + 1,
                                             nsga2Algorithm.childPop->generation,
                                             nsga2Algorithm.childPop->GetIndReals(scores.id + 1));
                        // nsga2Algorithm.childPop->EvaluateInd(scores.id + 1);
                    }
                    else if (scores.id == nsga2Algorithm.popSize - 1) {
                        if (scores.generation == nsga2Algorithm.generations)  // FINAL GENERATION
                        {
                            nsga2Algorithm.PostEvaluationAdvance();
                            nsga2Algorithm.ReportFinalGenerationPop();
                            std::cout << "NSGA2 evaluation finished!" << std::endl;
                            std::unique_ptr<NSGA2Terminate> terminate = std::make_unique<NSGA2Terminate>();
                            terminate->terminateMe                    = 1;
                            emit(terminate);
                            // send command to terminate tests
                        }
                        else if (scores.generation < nsga2Algorithm.generations) {
                            nsga2Algorithm.PostEvaluationAdvance();
                            nsga2Algorithm.PreEvaluationAdvance();

                            std::cout << "Evaluating gen " << nsga2Algorithm.childPop->generation << " ind 0"
                                      << std::endl;
                            requestIndEvaluation(
                                0, nsga2Algorithm.childPop->generation, nsga2Algorithm.childPop->GetIndReals(0));
                        }
                    }
                }
            });
        }

        void NSGA2Optimiser::requestIndEvaluation(int _id, int _generation, const std::vector<double>& _reals) {
            std::unique_ptr<NSGA2EvaluationRequest> request = std::make_unique<NSGA2EvaluationRequest>();
            request->id                                     = _id;
            request->generation                             = _generation;
            request->reals                                  = _reals;
            emit(request);  // send to the kick engine all the _reals for the population
        }
    }  // namespace optimisation
}  // namespace support
}  // namespace module
