#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

namespace module {
    namespace support {
        namespace optimisation {

            class NSGA2Evaluator : public NUClear::Reactor {

            public:
                /// @brief Called by the powerplant to build and setup the NSGA2Evaluator reactor.
                explicit NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment);

            private:
                void SendFitnessScores();
                void ResetWorld();
                void ResetWorldTime();
                void BeginTermination();
                void CalculateFitness();

                size_t subsumptionId;
                int generation                   = 0;
                int id                           = 0;
                std::vector<double> scores       = {2.0, 0.0};
                std::vector<double> contstraints = {0.0, 0.0};

                double simTime      = 0.0;
                double simTimeDelta = 0.0;
                double timeSinceTermination;
                Eigen::Vector2d velocity = Eigen::Vector2d(0.5, 0.0);
                double rotation          = 0.0;

                bool terminating = false;
                bool walking     = false;
                bool standing    = false;
                bool evaluating  = false;
                bool finished    = false;
                bool fallenOver  = false;

                std::vector<double> constraints;

                Eigen::Vector3d gyroscope     = Eigen::Vector3d::Zero();
                Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();

                Eigen::Vector3d ballLocation       = Eigen::Vector3d::Zero();
                Eigen::Vector3d ballVelocity       = Eigen::Vector3d::Zero();
                Eigen::Vector3d robotLocation      = Eigen::Vector3d::Zero();
                Eigen::Vector3d robotLocationDelta = Eigen::Vector3d::Zero();

                double distanceTravelled = 0.0;

                Eigen::Vector3d sway = Eigen::Vector3d::Zero();

                double fieldPlaneSway = 0.0;

                double maxFieldPlaneSway;
            };

        }  // namespace optimisation
    }      // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
