// Interface classes for data exchange between the generic gait motion module and gait engines
// File: gait_interface.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_INTERFACE_H
#define GAIT_INTERFACE_H

#include "GaitCommand.h"

#include <Eigen/Geometry>

#include "utility/input/ServoID.h"

namespace gait {
//! Enumeration of motion stances that can be commanded to a gait engine
enum MotionStance { STANCE_DEFAULT, STANCE_KICK, STANCE_COUNT };

/**
 * @brief Data struct for passing gait engine input data from the
 * generic gait motion module to the gait engine that it manages.
 **/
struct GaitEngineInput {
    //! Default constructor
    GaitEngineInput() {
        reset();
    }

    //! Reset function
    void reset() {
        timestamp             = 0.0;
        nominaldT             = 0.0;
        truedT                = 0.0;
        motionPending         = false;
        motionAdjustLeftFoot  = false;
        motionAdjustRightFoot = false;
        jointPos.fill(0);
        gaitCmd.reset();
    }

    //! @brief Current measured position of each joint (indexed by the `JointID` enum, in `rad`).
    std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> jointPos;

    // System parameters
    //! @brief The current time in seconds (guaranteed to be monotonic increasing)
    double timestamp;
    //! @brief The nominal time between calls to the gait engine's `step()` function.
    double nominaldT;
    //! @brief The true time since the last call to the gait engine's `step()` function.
    //! This value is clamp to avoid spikes
    double truedT;


    // Gait command
    //! @brief Gait command (e.g. desired walking velocity and so on).
    GaitCommand gaitCmd;

    // Motion parameters
    //! @brief Boolean flag whether a motion is pending.
    bool motionPending;
    //!< @brief The stopping stance required for the playing of the pending motion.
    MotionStance motionStance;
    //! @brief Boolean flag whether the left foot should be used to adjust the stopping stance.
    bool motionAdjustLeftFoot;
    //! @brief Boolean flag whether the right foot should be used to adjust the stopping stance.
    bool motionAdjustRightFoot;

    //! @brief world to torso transformation.
    Eigen::Affine3d Htw;
    //! @brief gyroscope sensor value.
    Eigen::Vector3d gyroscope;
};

/**
 * @brief Data struct for passing gait engine output data from the
 * gait engine to the generic gait motion module that manages it.
 **/
struct GaitEngineOutput {
    //! Default constructor
    GaitEngineOutput() {
        reset();
    }

    //! Reset function
    void reset() {
        jointCmd.fill(0);
        jointEffort.fill(0);
        odomPosition.fill(0);
        useRawJointCmds      = false;
        walking              = false;
        supportCoeffLeftLeg  = 0.0;
        supportCoeffRightLeg = 0.0;
        odomOrientation      = {1, 0, 0, 0};
    }

    // Joint commands
    //! @brief Commanded position for each joint (indexed by the `JointID` enum, in `rad`).
    std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> jointCmd;
    //! @brief Commanded joint effort (indexed by the `JointID` enum, in the range `[0,1]`).
    std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> jointEffort;
    //! @brief Apply the joint commands directly to the hardware, without using compensation or actuator controller(s)
    //! in-between.
    bool useRawJointCmds;

    // Status flags
    //! @brief Flag specifying whether the gait is currently active and walking (`true`) or halted (`false`).
    bool walking;

    // Support coefficients
    //! @brief Current support coefficient of the left leg.
    double supportCoeffLeftLeg;
    //! @brief Current support coefficient of the right leg.
    double supportCoeffRightLeg;

    // Robot odometry transform
    //! @brief Position `(x,y,z)` of the robot's body-fixed base transform (centred at the robot's centre of mass) in
    //! global odometry coordinates.
    std::array<double, 3> odomPosition;
    //! @brief Orientation `(w,x,y,z)` of the robot's body-fixed base transform (centred at the robot's centre of mass)
    //! relative to the global odometry frame.
    std::array<double, 4> odomOrientation;
};
}  // namespace gait

#endif  // GAIT_INTERFACE_H
