#ifndef MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_H
#define MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_H

#include <nuclear>
#include <vector>

#include "NSGA2Evaluator.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

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
                virtual void process_raw_sensor_msg(const RawSensors& sensors, NSGA2Evaluator* evaluator)   = 0;
                virtual void process_optimisation_robot_position(const OptimisationRobotPosition& position) = 0;
                virtual void set_up_trial(const NSGA2EvaluationRequest& request)                            = 0;
                virtual void reset_simulation()                                                             = 0;
                virtual void evaluating_state(size_t subsumptionId, NSGA2Evaluator* evaluator)              = 0;
                virtual std::unique_ptr<NSGA2FitnessScores> calculate_fitness_scores(bool earlyTermination,
                                                                                     double simTime,
                                                                                     int generation,
                                                                                     int individual)        = 0;
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_EVALUATORTASK_H
