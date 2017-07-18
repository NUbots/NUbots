#ifndef LIMPMODEL_H_
#define LIMPMODEL_H_

#include "Limp.h"
#include "LimpState.h"

#include <Eigen/Core>

namespace gait {
namespace contrib {

    struct LimpModel {
        double systemIterationTime;
        double fusedAngleX;
        double fusedAngleY;

        //  char name;
        Eigen::Vector3f gcv;  // input

        // current state
        double x;
        double vx;
        double ax;
        double y;
        double vy;
        double ay;
        double energyX;
        double energyY;
        int supportLegSign;
        bool crossing;
        bool stalling;

        LimpState nominalState;
        Eigen::Vector2f nominalCapturePoint;
        Eigen::Vector3f nominalFootStep;
        Eigen::Vector3f nominalStepSize;
        double nominalFootStepTHalf;

        double timeSinceStep;
        double timeToStep;
        double nominalTimeToStep;

        Eigen::Vector2f zmp;
        LimpState endOfStepState;
        Eigen::Vector3f footStep;
        Eigen::Vector3f stepSize;

        Eigen::Vector2f rxZmp;
        Eigen::Vector2f rxCapturePoint;

    private:
        Limp limp;

    public:
        explicit LimpModel();

        void reset() {
            LimpState ms;
            ms.reset();  // Just to be safe
            setState(ms,
                     Eigen::Vector3f::Zero());  // TODO: This is probably not such a good idea as a zero LimpState is
                                                // actually highly atypical and a situation that is in unstable
                                                // equilibrium, leading to possibly weird timeToStep's and so on.
            updateInputData(systemIterationTime, 0.0, 0.0);
        }

        inline void updateInputData(double systemIterationTime, double fusedAngleX, double fusedAngleY) {
            this->systemIterationTime = systemIterationTime;
            this->fusedAngleX         = fusedAngleX;
            this->fusedAngleY         = fusedAngleY;
        }

        void setState(LimpState ms, Eigen::Vector3f ggcv);
        bool forwardThroughStep(double dt);
        void forward(double dt);
        LimpModel forwarded(double t);
        void step();  // Make a footstep in the model

        double timeToLoc(double loc);
        double timeToSel();
        double timeToApex();

        LimpState mirroredToRight();
        LimpState getMotionState();
        LimpState getNominalState();
        LimpState getEndOfStepState();
        Eigen::Vector2f getCapturePoint();

        inline bool operator==(const LimpModel& v) const {
            return (x == v.x) && (vx == v.vx) && (y == v.y) && (vy == v.vy);
        }
        inline bool operator!=(const LimpModel& v) const {
            return (x != v.x) || (vx != v.vx) || (y != v.y) || (vy != v.vy);
        }
    };

}  // namespace contrib
}  // namespace gait

#endif
