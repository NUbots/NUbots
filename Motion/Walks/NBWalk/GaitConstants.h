#ifndef GaitConstants_h_DEFINED
#define GaitConstants_h_DEFINED
#include "NBInclude/NBMath.h"


/**
 * Checklist of things you need to update when adding an attribute to a
 * pre-existing Config:
 *IN THIS FILE:
 *  - Add it to the enum. make sure to leave the LEN_XXX_CONFIG as the last
 *    item in the enum
 *  - Add the correct python->c++ conversion constant (e.g. LENGTH,ANGLE,NONE)
 *  - Add a reasonable default to initialize C instances incase walking is
 *    called without getting Gaits sent from Python first.
 *IN ROBOTGAITS
 *  - Fix the tuple lengths to match correctly
 */

/**
 * Checklist of things you need to update when adding a new config:
 *  - Add the enum, the conversion, and the default arrays in this file.
 *  - Update AbstractGait.h/cpp to make sure the constructors and element
 *    copying will include the new array associated with your config
 *  - Maybe other things
 */

namespace WP {
/**
 * STANCE CONFIG holds the following parameters:
 *  bodyHeight      -- CoM Height
 *  bodyOffsetY     -- Forward displacement of CoM
 *  legSeparationY  -- Horizontal distance between feet
 *  bodyRotationY   -- Body angle around Y axis
 *  legRotationY    -- Angle between feet (around Z axis)
 *
 */
    enum StanceConfig {
        BODY_HEIGHT=0,
        BODY_OFF_X,
        LEG_SEPARATION_Y,
        BODY_ROT_Y,
        LEG_ROT_Z,
        TRANS_TIME,
        LEN_STANCE_CONFIG
    };

/**
 * STEP CONFIG
 *  stepDuration         -- time allocated for each step
 *  dblSupportPercent   -- fraction of time in double support
 *  stepHeight           -- step height during a step
 *  maxVelX              -- maximum forward velocity of step
 *  maxVelY              -- maximum lateral velocity of step
 *  maxVelTheta          -- maximum angular velocity of step
 */
    enum StepConfig{
        DURATION,
        DBL_SUPP_P,
        STEP_HEIGHT, //TODO move this to STANCE
        FOOT_LIFT_ANGLE,
        MAX_VEL_X,
        MIN_VEL_X,
        MAX_VEL_Y,
        MAX_VEL_THETA,
        MAX_ACC_X,
        MAX_ACC_Y,
        MAX_ACC_THETA,
        WALKING, //1.0 is walking, everything else is not walking.
        LEN_STEP_CONFIG
    };

    static const float NON_WALKING_GAIT = 0.0f;
    static const float WALKING_GAIT = 1.0f;

/**
 * ZMP CONFIG holds the following parameters:
 *  footCenterX                   -- footLengthX
 *  doubleSupportStaticPercentage -- zmp static percentage
 *  lZMPOffY                -- left zmp off
 *  rZMPOffY               -- right zmp off
 *  strafeZMPOff               -- turn zmp offset
 *  turnZMPOff                 -- strafe zmp offset
 *
 */
    enum ZmpConfig{
        FOOT_CENTER_X=0,
        DBL_SUP_STATIC_P,
        L_ZMP_OFF_Y,
        R_ZMP_OFF_Y,
        STRAFE_ZMP_OFF,
        TURN_ZMP_OFF,
        LEN_ZMP_CONFIG
    };

/**
 * JOINT HACK CONFIG
 *  lHipAmplitude  -- magnitude of angular addition to hip during step
 *  rHipAmplitude -- magnitude of angular addition to hip during step
 */
    enum JointHackConfig{
        L_HIP_AMP=0,
        R_HIP_AMP,
        LEN_HACK_CONFIG
    };

/**
 * SENSOR CONFIG
 * observerScale   -- proportion of observer feedback
 * angleScale      -- proportion of angleXY feedback
 */
    enum SensorConfig{
        FEEDBACK_TYPE=0, //This is a bit bad, since we don't want to interpolate
        GAMMA_X,
        GAMMA_Y,
        SPRING_K_X,
        SPRING_K_Y,
        MAX_ANGLE_X,
        MAX_ANGLE_Y,
        MAX_ANGLE_VEL,
        LEN_SENSOR_CONFIG
    };

/**
 * STIFFNESS CONFIG
 * hipStiff     -- stiffnesses for the hip
 * KPStiff      -- stiffnesses for knee pitch
 * APStiff      -- stiffnesses for ankle pitch
 * ARStiff      -- stiffnesses for ankle roll
 * armStiff     -- stiffnesses for the arms
 */
    enum StiffnessConfig{
        HIP = 0,
        KP,
        AP,
        AR,
		ARM,
        ARM_PITCH,
        LEN_STIFF_CONFIG
    };

/**
 * ODO CONFIG
 * xOdoScale  -- odometry calibration for forward direction
 * yOdoScale  -- odometry calibration for lateral direction
 * thetaOdoScale  -- odometry calibration for rotational direction
 */
    enum OdoConfig{
        X_SCALE = 0,
        Y_SCALE,
        THETA_SCALE,
        LEN_ODO_CONFIG
    };

/**
 * ARM CONFIG
 * armAmplitude -- angle amplitude of arm motion
 */
    enum ArmConfig{
        AMPLITUDE = 0,
        LEN_ARM_CONFIG
    };

    static const float STANCE_DEFAULT[LEN_STANCE_CONFIG]=
    {310.0f,        // com height (mm)
     10.0f,         // forward displacement of CoM (mm)
     115.0f,        // foot separation (mm)
     0.05f,         // forward lean angle (rad)                     ---> Walk Parameter
     0.0f,          // yaw foot angle (rad)
     0.1f};         // transition time (s)
    static const float STEP_DEFAULT[LEN_STEP_CONFIG]=
    {0.4f,          // step time (s)                                ---> Walk Parameter
     0.25f,         // fraction of time spent in double support     ---> Walk Parameter
     15.0f,         // the step height (mm)                         ---> Walk Parameter
     0.0f,          // angle of the foot while lifted (rad)         ---> Walk Parameter
     70.0f,        // max forward vel x (cm/s)                      ---> All of the velocities and accelerations are Walk Parameters
     -70.0f,        // max backward vel x (cm/s)
     70.0f,        // max vel y (cm/s)
     0.35f,         // max vel yaw (rad/s) 
     70.0f,         // max acc x (cm/s/s)
     70.0f,         // max acc y (cm/s/s)
     0.35f,         // max acc t (rad/s)
     WALKING_GAIT}; // Walking or not
    static const float ZMP_DEFAULT[LEN_ZMP_CONFIG]=
    {0.0f,          // foot center
     0.4f,          // zmp static perc                              ---> Walk Parameter
     10.0f,          // l zmp off                                   ---> Walk Parameter
     10.0f,          // r zmp off                                   ---> Walk Parameter 
     0.01f,          // strafe zmp off
     0.12f,};        // turn zmp off  
    static const float HACK_DEFAULT[LEN_HACK_CONFIG]=
    {0.1f,          // hip hack l                                   ---> Walk Parameter
     0.1f};         // hip hack r                                   ---> Walk Parameter
    static const float SENSOR_DEFAULT[LEN_SENSOR_CONFIG]=
    {0.0,   // Feedback type (1.0 = spring, 0.0 = old)
     0.06,  // angle X scale (gamma)
     0.08,  // angle Y scale (gamma)
     250.0,  // X spring constant k (kg/s^2)
     100.0,  // Y spring constant k (kg/s^2)
     0.122,   // max angle X (compensation)
     0.122,   // max angle Y
     0.785};   // max acceleration
    static const float STIFF_DEFAULT[LEN_STIFF_CONFIG]=
    {0.85f,//hip
     0.3f,//knee
     0.4f,//ap
     0.3f,//ar
     0.2f,//arm
	 0.2f};//arm pitch
    static const float ODO_DEFAULT[LEN_ODO_CONFIG]=
    {1.0f,//xodoscale
     1.0f,//yodoscale
     1.0f};//thetaodoscale
    static const float ARM_DEFAULT[LEN_ARM_CONFIG]=
    {0.17f};//arm amplitude

};//End namespace WP
#endif
