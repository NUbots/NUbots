#ifndef MODULE_SUPPORT_OPTIMISATION_STANDEVALUATOR_H
#define MODULE_SUPPORT_OPTIMISATION_STANDEVALUATOR_H

#include <Eigen/Core>
#include <nuclear>
#include <vector>

#include "EvaluatorTask.hpp"

#include "extension/Script.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            using message::platform::RawSensors;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;

            class StandEvaluator : public EvaluatorTask {
            public:
                // Implementing the EvaluatorTask interface
                void processRawSensorMsg(const RawSensors& sensors, NSGA2Evaluator* evaluator);
                void processOptimisationRobotPosition(const OptimisationRobotPosition& position);
                void setUpTrial(const NSGA2EvaluationRequest& request);
                void resetSimulation();
                void evaluatingState(size_t subsumptionId, NSGA2Evaluator* evaluator);
                std::unique_ptr<NSGA2FitnessScores> calculateFitnessScores(bool earlyTermination,
                                                                           double sim_time,
                                                                           int generation,
                                                                           int individual);

                // Task-specific functions
                std::vector<double> calculateScores(double trialDuration);
                std::vector<double> calculateConstraints();
                bool checkForFall(const RawSensors& sensors);
                void updateMaxFieldPlaneSway(const RawSensors& sensors);

            private:
                /// @brief Robot state for this evaluation, used during fitness and constraint calculation
                Eigen::Vector3d robot_position = Eigen::Vector3d::Zero();
                double max_field_plane_sway    = 0.0;
                RawSensors current_sensors;

                /// @brief The amount of time to run a single trial, in seconds.
                std::chrono::seconds trial_duration_limit = std::chrono::seconds(0);

                /// @brief Keep track of when the trial started
                double trial_start_time = 0.0;

                void loadScript(std::string script_path);
                void saveScript(std::string script_path);
                void runScript(size_t subsumptionId, NSGA2Evaluator* evaluator);

                /// @brief The script object we are using
                ::extension::Script script;
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_STANDEVALUATOR_H
