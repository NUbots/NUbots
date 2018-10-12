#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
#define MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H

#include <armadillo>
#include <nuclear>

#include "extension/Script.h"

namespace module {
namespace support {
    namespace optimisation {

        class NSGA2Evaluator : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the NSGA2Evaluator reactor.
            explicit NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment);
            void SendFitnessScores();
            void ResetWorld();
            void ResetWorldTime();
            bool BeginTermination();
            void CalculateFitness();

            size_t subsumptionId;
            int generation;
            int id;
            std::vector<double> scores;
            std::vector<double> contstraints;
            std::vector<double> parameters;

            double simTime;
            double simTimeDelta;
            double timeSinceTermination;
            arma::vec2 velocity;
            float rotation;

            bool terminating;
            bool walking;
            bool standing;
            bool evaluating;
            bool finished;
            bool fallenOver;

            std::vector<double> constraints;

            double gyroscope[3];      // [X, Y, Z]
            double accelerometer[3];  // [X, Y, Z]

            double ballLocation[3];  // [X, Y, Z]
            double ballVelocity[3];
            double robotLocation[3];
            double robotLocationDelta[3];

            double distanceTravelled;

            double sway[3];

            double fieldPlaneSway;


            double maxFieldPlaneSway;

            // the script being modified
            ::extension::Script script;
            bool optimizeScript;
        };

    }  // namespace optimisation
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2EVALUATOR_H
