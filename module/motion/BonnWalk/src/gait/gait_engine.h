// Base class for all gait engines
// File: gait_engine.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_ENGINE_H
#define GAIT_ENGINE_H

// Includes
#include <Eigen/Core>
#include <array>
#include "gait_interface.h"

// Gait namespace
namespace gait {

// Class forward declarations
class Gait;

/**
 * @class GaitEngine
 *
 * @brief Base implementation of a gait engine, made to work with the `Gait` motion module.
 **/
class GaitEngine {
public:
    //! Default constructor
    GaitEngine();

    /**
     * @brief Reset the gait engine.
     *
     * This function is used by the `Gait` class, in conjunction with the `resetBase()` function,
     * to reset the GaitEngine object in terms of both its derived and base components respectively.
     * This function must be able to clean up and reset the gait engine instance, no matter what
     * state it is currently in.
     **/
    void reset();

    /**
     * @brief Update the halt pose desired by the gait engine.
     *
     * This function should update the halt pose of the gait engine if/as required. The halt pose is
     * stored in three variables, namely #haltJointCmd, #haltJointEffort and #haltUseRawJointCmds.
     * This should be the robot pose from which the gait engine is nominally intended to be started
     * and stopped from. The halt pose should normally be relatively constant during execution, but
     * may for example depend on configuration parameters, and so is allowed to dynamically change.
     *
     * This function is intended for use by the gait engine itself as well, such as for example at
     * the beginning of the derived `step()` function override. Make no assumptions about when this
     * function is called externally.
     **/
    virtual void updateHaltPose();

    /**
     * @brief Main step function of the gait engine.
     *
     * The step function is called in every execution cycle that the gait is required to be active.
     * The command inputs for the gait engine can be retrieved from the `in` class member, which
     * ideally should only be read from, and the outputs of the gait engine should be written into
     * the `out` class member. Ideally *all* members of the `out` struct should be written to from
     * within the `step()` function, as the members may contain arbitrary values until written to.
     * The latest robot state information can be read from the `model` class member, which is a
     * const pointer to the required RobotModel object. The pointer is guaranteed to be valid before
     * the `step()` function is called for the first time.
     **/
    virtual void step();

    /**
     * @brief Set the CoM odometry to a particular 2D position and orientation.
     *
     * Specifies the required `(x,y)` position and yaw rotation for the CoM odometry. How exactly the
     * yaw parameter is interpreted in the light of additional pitch and roll rotations is up to the
     * implementation. It is recommended that the parameter is treated as fused yaw. Ideally, after
     * calling this function the next retrieval of the robot odometry should reveal exactly @p posX
     * and @p posY in the position vector (see `GaitEngineOutput::odomPosition`). If you override this
     * function then you need to override `updateOdometry` too.
     **/
    virtual void setOdometry(double posX, double posY, double rotZ);

    /**
     * @brief Force an update of the CoM odometry in terms of 3D position and orientation.
     *
     * This function should update the `GaitEngineOutput::odomPosition` and `GaitEngineOutput::odomOrientation`
     * members of the @c out member of the `GaitEngine` class. Most of the time these two data fields
     * will already be up-to-date anyway, but after calling this function the caller should be able
     * to take for granted that the odometry stored in @c out is up-to-date, and some valid value.
     * The default implementation simply writes the last set odometry (`setOdometry`) into the fields.
     **/
    virtual void updateOdometry();

    // Gait engine data interface structs
    GaitEngineInput in;    //!< Gait engine input data struct.
    GaitEngineOutput out;  //!< Gait engine output data struct.

protected:
    /**
     * @brief Pointer to the RobotModel object to use for retrieving state information in each step.
     *
     * This parameter is guaranteed to be set by the `Gait` class prior to any other function being
     * called. For obvious reasons it is heavily discouraged for a gait engine to write to this variable.
     **/
    const robotcontrol::RobotModel* model;

    /**
     * @brief Reset the GaitEngine base class.
     *
     * This function is used by the `Gait` class, in conjunction with the virtual `reset()` function,
     * to reset the GaitEngine object in terms of both its base and derived components respectively.
     **/
    void resetBase();

    // Halt pose specification
    //! Commanded halt position for each joint (indexed by the `JointID` enum, in `rad`).
    std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> haltJointCmd;
    //!< Commanded halt joint effort (indexed by the `JointID` enum, in the range `[0,1]`).
    std::array<double, utility::input::ServoID::NUMBER_OF_SERVOS> haltJointEffort;
    //! Apply the joint commands directly to the hardware in the halt pose, without using compensation or actuator
    //! controller(s) in-between.
    bool haltUseRawJointCmds;

private:
    // Internal variables
    double m_posX;
    double m_posY;
    double m_rotZ;

    // Common motion data struct
    struct CommonMotionData {
        // Constructor
        // All data that is initialised here is just to avoid divisions by zero etc in case a bug
        // causes an uninitialised value in this struct to be used by accident
        CommonMotionData()
            : gcvX(0.0)
            , gcvY(0.0)
            , gcvZ(0.0)
            , absGcvX(0.0)
            , absGcvY(0.0)
            , absGcvZ(0.0)
            , gaitPhase(0.0)
            , oppGaitPhase(M_PI)
            , limbPhase(0.0)
            , absPhase(0.0)
            , doubleSupportPhase(0.1)
            , swingStartPhase(0.0)
            , swingStopPhase(M_PI)
            , suppTransStartPhase(0.0)
            , suppTransStopPhase(doubleSupportPhase)
            , liftingPhaseLen(M_PI - doubleSupportPhase)
            , suppPhaseLen(M_PI + doubleSupportPhase)
            , nonSuppPhaseLen(M_2PI - suppPhaseLen)
            , sinusoidPhaseLen(M_PI)
            , linearPhaseLen(M_PI)
            , swingAngle(0.0) {}

        // Gait command vector variables
        double gcvX;
        double gcvY;
        double gcvZ;
        double absGcvX;
        double absGcvY;
        double absGcvZ;

        // Gait phase variables
        double gaitPhase;
        double oppGaitPhase;
        double limbPhase;
        double absPhase;

        // Phase marks
        double doubleSupportPhase;
        double swingStartPhase;
        double swingStopPhase;
        double suppTransStartPhase;
        double suppTransStopPhase;

        // Extra swing variables
        double liftingPhaseLen;
        double suppPhaseLen;
        double nonSuppPhaseLen;
        double sinusoidPhaseLen;
        double linearPhaseLen;

        // Swing angle
        double swingAngle;
    };

    //
    // Private functions
    //

    // Reset walking function (a lighter version of reset(), used for internal resets when starting and stopping
    // walking)
    void resetWalking(bool walking, const Eigen::Vector3d& gcvBias);

    // Input processing
    void processInputs();
    void updateRobot(const Eigen::Vector3d& gcvBias);

    // Helper functions
    CommonMotionData calcCommonMotionData(bool isFirst) const;

    // Motion functions
    void abstractLegMotion(gait::AbstractLegPose& leg);
    void abstractArmMotion(gait::AbstractArmPose& arm);
    void inverseLegMotion(gait::InverseLegPose& leg);

    // Coercion functions
    void coerceAbstractPose(gait::AbstractPose& pose);
    void coerceAbstractArmPose(gait::AbstractArmPose& arm);
    void coerceAbstractLegPose(gait::AbstractLegPose& leg);

    // Output processing
    void updateOutputs();

    // Blending functions
    void resetBlending(double b = USE_HALT_POSE);
    void setBlendTarget(double target, double phaseTime);
    double blendFactor();

    // Motion stance functions
    void resetMotionStance();

    //
    // Gait variables
    //

    // Constants
    const std::string CONFIG_PARAM_PATH;
    static const double USE_HALT_POSE = 1.0;
    static const double USE_CALC_POSE = 0.0;

    // Gait configuration struct
    CapConfig config;

    // Pose variables
    gait::JointPose m_jointPose;            // Joint representation of the pose to command in a step
    gait::JointPose m_jointHaltPose;        // Joint representation of the gait halt pose
    gait::JointPose m_lastJointPose;        // The last joint pose to have been commanded during walking
    gait::InversePose m_inversePose;        // Inverse representation of the pose to command in a step
    gait::AbstractPose m_abstractPose;      // Abstract representation of the pose to command in a step
    gait::AbstractPose m_abstractHaltPose;  // Abstract representation of the gait halt pose

    // Gait command vector variables
    // Gait command velocity vector (slope-limited command velocities actually followed by the gait engine)
    Eigen::Vector3d m_gcv;
    // Gait command velocity vector input (raw velocities commanded by the gait motion module)
    Eigen::Vector3d m_gcvInput;
    // Derivative filter for the GCV to calculate the gait acceleration
    rc_utils::GolayDerivative<Eigen::Vector3d, 1, 5, Eigen::aligned_allocator<Eigen::Vector3d>> m_gcvDeriv;
    rc_utils::MeanFilter m_gcvAccSmoothX;
    rc_utils::MeanFilter m_gcvAccSmoothY;
    rc_utils::MeanFilter m_gcvAccSmoothZ;
    Eigen::Vector3d m_gcvAcc;

    // Gait flags
    bool m_walk;          // True if the gait engine should walk, false if it should not walk
    bool m_walking;       // True if the gait engine is currently producing a walking motion output
    bool m_leftLegFirst;  // True if the left leg should be the first to be lifted when starting walking

    // Step motion variables
    double m_gaitPhase;  // Current walking phase of the gait motion

    // Blending variables
    bool m_blending;
    double m_b_current;
    double m_b_initial;
    double m_b_target;
    double m_blendPhase;
    double m_blendEndPhase;

    // Motion stance variables
    // Interpolation factor between feet narrow (= 0) and feet normal (= 1) legAngleX values
    double m_motionLegAngleXFact;

    //
    // Basic feedback variables
    //

    // Basic feedback filters
    rc_utils::MeanFilter fusedXFeedFilter;
    rc_utils::MeanFilter fusedYFeedFilter;
    rc_utils::WLBFFilter dFusedXFeedFilter;
    rc_utils::WLBFFilter dFusedYFeedFilter;
    rc_utils::MeanFilter iFusedXFeedFilter;
    rc_utils::MeanFilter iFusedYFeedFilter;
    rc_utils::WLBFFilter gyroXFeedFilter;
    rc_utils::WLBFFilter gyroYFeedFilter;

    // Integrators
    rc_utils::EWIntegrator iFusedXFeedIntegrator;
    rc_utils::EWIntegrator iFusedYFeedIntegrator;
    config_server::Parameter<bool> m_resetIntegrators;  // Rising edge triggered flag to reset any integrated or learned
                                                        // values in the gait that are not necessarily reset during
                                                        // start/stop of walking
    config_server::Parameter<bool> m_saveIFeedToHaltPose;  // Rising edge triggered flag to save the current integrated
                                                           // feedback values as offsets to the halt pose (only the ones
                                                           // in current use)
    bool m_savedLegIFeed;    // Flag that specifies within a cycle whether the integrated leg feedback has already been
                             // saved
    bool m_savedArmIFeed;    // Flag that specifies within a cycle whether the integrated arm feedback has already been
                             // saved
    double iFusedXLastTime;  // The last in.timestamp where iFusedX made a non-zero contribution to the CPG gait
    double iFusedYLastTime;  // The last in.timestamp where iFusedY made a non-zero contribution to the CPG gait
    bool haveIFusedXFeed;    // Boolean flag whether the iFusedX integrator had any effect on iFusedXFeed in the current
                             // cycle
    bool haveIFusedYFeed;    // Boolean flag whether the iFusedY integrator had any effect on iFusedYFeed in the current
                             // cycle
    bool usedIFusedX;  // Boolean flag whether the iFusedX integrator had any effect on the produced joint commands in
                       // the current cycle
    bool usedIFusedY;  // Boolean flag whether the iFusedY integrator had any effect on the produced joint commands in
                       // the current cycle
    void resetIntegrators();
    void resetSaveIntegrals();

    // Callbacks for updating the filter sizes
    void resizeFusedFilters(int numPoints) {
        if (numPoints < 1) numPoints = 1;
        fusedXFeedFilter.resize(numPoints);
        fusedYFeedFilter.resize(numPoints);
    }
    void resizeDFusedFilters(int numPoints) {
        if (numPoints < 1) numPoints = 1;
        dFusedXFeedFilter.resize(numPoints);
        dFusedYFeedFilter.resize(numPoints);
    }
    void resizeIFusedFilters(int numPoints) {
        if (numPoints < 1) numPoints = 1;
        iFusedXFeedFilter.resize(numPoints);
        iFusedYFeedFilter.resize(numPoints);
    }
    void resizeGyroFilters(int numPoints) {
        if (numPoints < 1) numPoints = 1;
        gyroXFeedFilter.resize(numPoints);
        gyroYFeedFilter.resize(numPoints);
    }

    // Basic feedback values
    double fusedXFeed;
    double fusedYFeed;
    double dFusedXFeed;
    double dFusedYFeed;
    double iFusedXFeed;
    double iFusedYFeed;
    double gyroXFeed;
    double gyroYFeed;

    //
    // Capture step variables
    //

    // Reset function
    void resetCaptureSteps(bool resetRobotModel);

    // Capture step robot model
    contrib::RobotModel rxRobotModel;
    contrib::RobotModelVis m_rxVis;
    config_server::Parameter<bool> m_showRxVis;
    void callbackShowRxVis();

    // Linear inverted pendulum robot models
    contrib::LimpModel rxModel;
    contrib::LimpModel mxModel;
    contrib::LimpModel txModel;

    // Linear inverted pendulum model class for isolated calculations
    contrib::Limp limp;

    // CoM filter
    ComFilter<5> m_comFilter;

    // Miscellaneous
    config_server::Parameter<float> m_gcvZeroTime;
    contrib::Vec2f adaptation;
    double lastSupportOrientation;
    double oldGcvTargetY;
    double virtualSlope;
    double stepTimeCount;
    double lastStepDuration;
    int stepCounter;
    int noCLStepsCounter;
    int resetCounter;
    int cycleNumber;
};

}  // namespace gait

#endif /* GAIT_ENGINE_H */
