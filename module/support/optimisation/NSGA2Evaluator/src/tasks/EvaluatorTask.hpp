#ifndef MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_H
#define MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_H

#include <nuclear>
#include <vector>

#include "NSGA2Evaluator.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
#include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::platform::RawSensors;
            using message::platform::webots::OptimisationRobotPosition;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;

            class NSGA2Evaluator;  // forward declaration

            class EvaluatorTask {
            public:
                virtual void processRawSensorMsg(const RawSensors& sensors, NSGA2Evaluator* evaluator)   = 0;
                virtual void processOptimisationRobotPosition(const OptimisationRobotPosition& position, NSGA2Evaluator* evaluator) = 0;
                virtual void setUpTrial(const NSGA2EvaluationRequest& request)                           = 0;
                virtual void resetSimulation()                                                           = 0;
                virtual void evaluatingState(size_t subsumptionId, NSGA2Evaluator* evaluator)            = 0;
                virtual std::unique_ptr<NSGA2FitnessScores> calculateFitnessScores(bool earlyTermination,
                                                                                   double simTime,
                                                                                   int generation,
                                                                                   int individual)       = 0;
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_H
