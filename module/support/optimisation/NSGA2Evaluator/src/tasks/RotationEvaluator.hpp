#ifndef MODULE_SUPPORT_OPTIMISATION_ROTATIONVALUATOR_H
    #define MODULE_SUPPORT_OPTIMISATION_ROTATIONEVALUATOR_H

    #include <Eigen/Core>
    #include <nuclear>
    #include <vector>

    #include "EvaluatorTask.hpp"

    #include "message/platform/RawSensors.hpp"
    #include "message/support/optimisation/NSGA2EvaluatorMessages.hpp"
    #include "message/support/optimisation/NSGA2OptimiserMessages.hpp"

namespace module {
    namespace support {
        namespace optimisation {
            // using message::input::Sensors;
            using message::platform::RawSensors;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2FitnessScores;

            class RotationEvaluator : public EvaluatorTask {
            public:
                // Implementing the EvaluatorTask interface
                void processRawSensorMsg(const RawSensors& sensors, NSGA2Evaluator* evaluator);
                void processOptimisationRobotPosition(const OptimisationRobotPosition& position);
                void setUpTrial(const NSGA2EvaluationRequest& request);
                void resetSimulation();
                void evaluatingState(size_t subsumptionId, NSGA2Evaluator* evaluator);
                std::unique_ptr<NSGA2FitnessScores> calculateFitnessScores(bool earlyTermination,
                                                                           double simTime,
                                                                           int generation,
                                                                           int individual);

                // Task-specific functions
                std::vector<double> calculateScores();
                std::vector<double> calculateConstraints(double simTime);
                std::vector<double> constraintsNotViolated();
                bool checkForFall(const RawSensors& sensors);
                void updateMaxFieldPlaneSway(const RawSensors& sensors);

            private:
                /// @brief Robot state for this evaluation, used during fitness and constraint calculation
                bool initialPositionSet              = false;
                Eigen::Vector3d initialRobotPosition = Eigen::Vector3d::Zero();
                Eigen::Vector3d robotPosition        = Eigen::Vector3d::Zero();
                double maxFieldPlaneSway             = 0.0;

                /// @brief The amount of time to run a single trial, in seconds.
                std::chrono::seconds trial_duration_limit = std::chrono::seconds(0);

                /// @brief Keep track of when the trial started
                double trialStartTime = 0.0;

                /// @brief The walk command velocity.
                Eigen::Vector2d walk_command_velocity = Eigen::Vector2d(0.0, 0.0);

                /// @brief The walk command rotation.
                double walk_command_rotation = 0.0;

                double theta   = 0.0;
                double omega   = 0.0;
                double deltaT  = 0.0;
                double oldTime = 0.0;
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_ROTATIONEVALUATOR_H
