#ifndef GAIT_WALKCONFIG_H
#define GAIT_WALKCONFIG_H

#include <functional>
#include <vector>

namespace gait {

// clang-format off
struct WalkConfig {
    float armLinkLength;     //!< @brief Length of each/both the upper and lower arm links
    float legLinkLength;     //!< @brief Length of each/both the upper and lower leg links
    float shoulderWidth;     //!< @brief Horizontal separation between the two hip joints (length of the hip line)
    float hipWidth;          //!< @brief Horizontal separation between the two hip joints (length of the hip line)
    float trunkHeight;       //!< @brief Vertical height of the trunk from the hip line to the shoulder line
    float trunkLinkOffsetX;  //!< @brief Forward offset of the trunk link tf frame from the hip midpoint
    float trunkLinkOffsetY;  //!< @brief Leftward offset of the trunk link tf frame from the hip midpoint
    float trunkLinkOffsetZ;  //!< @brief Upward offset of the trunk link tf frame from the hip midpoint
    float comOffsetX;        //!< @brief Forward offset of the CoM in front of the hip line
    float comOffsetZ;        //!< @brief Height of the CoM above the hip line
    // float footWidth;   //!< @brief Width of the robot foot (along y-axis, the foot plate is assumed to be rectangular)
    // float footLength;  //!< @brief Length of the robot foot (along x-axis, the foot plate is assumed to be rectangular)
    // float footOffsetX;  //!< @brief Backward offset from the foot plate geometric center to the ankle joint (along x-axis)
    // float footOffsetY;  //!< @brief Inward offset from the foot plate geometric center to the ankle joint (along y-axis)
    float footOffsetZ;  //!< @brief Upward offset from the foot plate geometric center to the ankle joint (along z-axis)
    float neckHeight;   //!< @brief Height of the neck joint vertically above the center of the shoulder line (not for analytic calculation)
    float headOffsetX;  //!< @brief Forward offset from the neck joint to the center of the head (not for analytic calculation)
    float headOffsetZ;  //!< @brief Upward offset from the neck joint to the center of the head (not for analytic calculation)
    // ///@}

    // //! @name Robot model parameters
    // ///@{
    float footHeightHysteresis;  //!< @brief The minimum required foot height difference in the robot model to unlock the possibility of the model performing a support exchange
    // ///@}

    // //! @name General gait parameters
    // ///@{
    // TODO we don't have motion stances
    bool enableMotionStances;  //!< @brief Boolean flag whether to enable the use of motion stances (changes to the halt pose during stopping to allow a particular follow-up motion to be played)
    float gaitFrequency;       //!< @brief Nominal frequency of the gait
    float gaitFrequencyMax;    //!< @brief Maximum allowed frequency of the gait
    bool leftLegFirst;  //!< @brief Flag specifying whether the first leg to step with when starting walking should be the left leg
    float stanceAdjustGcvMax;  //!< @brief The maximum GCV at which stance adjustments are allowed to occur during stopping
    float stanceAdjustRate;  //!< @brief The dimensionless rate at which motion stance adjustment occurs while stopping walking
    float stoppingGcvMag;    //!< @brief Unbiased gait command velocity 2-norm below which immediate walk stopping is allowed
    float stoppingPhaseTolLB;  //!< @brief Gait phase tolerance below 0 and &pi;, in units of nominal phase increments (see gaitFrequency and the robotcontrol cycle time), within which intelligent walking stopping is allowed
    float stoppingPhaseTolUB;  //!< @brief Gait phase tolerance above 0 and -&pi;, in units of nominal phase increments (see gaitFrequency and the robotcontrol cycle time), within which intelligent walking stopping is allowed
    float supportCoeffRange;   //!< @brief The required difference between the symmetric support coefficients during walking (e.g. if this is 0.8 the support coefficients transition between 0.1 and 0.9, as 0.9 - 0.1 = 0.8 and 0.9 + 0.1 = 1.0)
    // TODO work out what this was
    bool useServoModel;  //!< @brief Flag specifying whether to use raw joint commands (i.e. no servo model, `false`), or use the servo model (`true`)
    // ///@}

    // //! @name Gait command vector parameters
    // ///@{
    float gcvBiasLinVelX;  //!< @brief Bias added to the linear x-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
    float gcvBiasLinVelY;  //!< @brief Bias added to the linear y-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
    float gcvBiasAngVelZ;  //!< @brief Bias added to the angular z-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
    float gcvAccForwards;  //!< @brief Gait command acceleration limit for positive linear x command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
    float gcvAccBackwards;   //!< @brief Gait command acceleration limit for negative linear x command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
    float gcvAccSidewards;   //!< @brief Gait command acceleration limit for linear y command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
    float gcvAccRotational;  //!< @brief Gait command acceleration limit for angular z command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
    float gcvDecToAccRatio;  //!< @brief Multiplicative scale factor to obtain the gait command deceleration limits from the gait command acceleration limits
    float gcvAccJerkLimitX;  //!< @brief Jerk (acceleration slope) limit for the calculation of the gait command acceleration from m_gcv (linear X direction)
    float gcvAccJerkLimitY;  //!< @brief Jerk (acceleration slope) limit for the calculation of the gait command acceleration from m_gcv (linear Y direction)
    float gcvAccJerkLimitZ;  //!< @brief Jerk (acceleration slope) limit for the calculation of the gait command acceleration from m_gcv (angular Z direction)
    float gcvPrescalerLinVelX;  //!< @brief Prescaler for the gcv linear velocity X that is then used by the open loop gait (allows easy scaling of the dynamic range of the gait)
    float gcvPrescalerLinVelY;  //!< @brief Prescaler for the gcv linear velocity Y that is then used by the open loop gait (allows easy scaling of the dynamic range of the gait)
    float gcvPrescalerAngVelZ;  //!< @brief Prescaler for the gcv angular velocity Z that is then used by the open loop gait (allows easy scaling of the dynamic range of the gait)
    // ///@}

    // //! @name Limb limit parameters
    // ///@{
    float limArmAngleXBuf;  //!< @brief Angle buffer for soft limiting of the arm angle X to be commanded by the gait (see `rc_utils::coerceSoft`)
    float limArmAngleXMax;  //!< @brief Maximum allowed arm angle X to be commanded by the gait (for left arm)
    float limArmAngleXMin;  //!< @brief Minimum allowed arm angle X to be commanded by the gait (for left arm)
    bool limArmAngleXUseLimits;  //!< @brief Boolean flag whether to use the specified arm angle X min/max limits
    float limArmAngleYBuf;  //!< @brief Angle buffer for soft limiting of the arm angle Y to be commanded by the gait (see `rc_utils::coerceSoft`)
    float limArmAngleYMax;  //!< @brief Maximum allowed arm angle Y to be commanded by the gait
    float limArmAngleYMin;  //!< @brief Minimum allowed arm angle Y to be commanded by the gait
    bool limArmAngleYUseLimits;  //!< @brief Boolean flag whether to use the specified arm angle Y min/max limits
    float limFootAngleXBuf;  //!< @brief Angle buffer for soft limiting of the foot angle X to be commanded by the gait (see `rc_utils::coerceSoft`)
    float limFootAngleXMax;  //!< @brief Maximum allowed foot angle X to be commanded by the gait (for left foot)
    float limFootAngleXMin;  //!< @brief Minimum allowed foot angle X to be commanded by the gait (for left foot)
    bool limFootAngleXUseLimits;  //!< @brief Boolean flag whether to use the specified foot angle X min/max limits
    float limFootAngleYBuf;  //!< @brief Angle buffer for soft limiting of the foot angle Y to be commanded by the gait (see `rc_utils::coerceSoft`)
    float limFootAngleYMax;  //!< @brief Maximum allowed foot angle Y to be commanded by the gait
    float limFootAngleYMin;  //!< @brief Minimum allowed foot angle Y to be commanded by the gait
    bool limFootAngleYUseLimits;  //!< @brief Boolean flag whether to use the specified foot angle Y min/max limits
    // float limLegAngleXBuf;  //!< @brief Angle buffer for soft limiting of the leg angle X to be commanded by the gait (see `rc_utils::coerceSoft`)
    float limLegAngleXMax;  //!< @brief Maximum allowed leg angle X to be commanded by the gait (for left leg)
    float limLegAngleXMin;  //!< @brief Minimum allowed leg angle X to be commanded by the gait (for left leg)
    bool limLegAngleXUseLimits;  //!< @brief Boolean flag whether to use the specified leg angle X min/max limits
    // float limLegAngleYBuf;  //!< @brief Angle buffer for soft limiting of the leg angle Y to be commanded by the gait (see `rc_utils::coerceSoft`)
    float limLegAngleYMax;  //!< @brief Maximum allowed leg angle Y to be commanded by the gait
    float limLegAngleYMin;  //!< @brief Minimum allowed leg angle Y to be commanded by the gait
    bool limLegAngleYUseLimits;  //!< @brief Boolean flag whether to use the specified leg angle Y min/max limits
    float limLegExtBuf;       //!< @brief Buffer for soft limiting of the leg extension to be commanded by the gait (see `rc_utils::coerceSoftMin`)
    float limLegExtMin;       //!< @brief Minimum allowed leg extension to be commanded by the gait
    bool limLegExtUseLimits;  //!< @brief Boolean flag whether to use the specified leg extension min limit
    // ///@}

    // //! @name Gait phase parameters
    // ///@{
    float startBlendPhaseLen;  //!< @brief The amount of time, in terms of gait phase, to take to blend from the halt pose to the moving calculated gait pose during start of walking
    float stopBlendPhaseLen;   //!< @brief The amount of time, in terms of gait phase, to take to blend from the moving calculated gait pose to the halt pose after cessation of walking
    float doubleSupportPhaseLen;  //!< @brief Length of the double support phase
    float swingStartPhaseOffset;  //!< @brief Offset from the end of the double support phase to the start of the swing phase
    float swingStopPhaseOffset;   //!< @brief Offset from the start of the double support phase back to the end of the previous swing phase (a positive value means the double support phase starts that amount after the end of swing)
    float swingMinPhaseLen;  //!< @brief Minimum allowed length of the swing phase (used as a check only, should have no effect on gait phase timing if there is no violation)
    float suppTransStartRatio;  //!< @brief Ratio that governs the start of support transitioning, via linear interpolation (0 => At start of double support phase, 1 => At end of immediately preceding end of swing phase)
    float suppTransStopRatio;  //!< @brief Ratio that governs the stop of support transitioning, via linear interpolation (0 => At end of double support phase, 1 => At start of immediately following start of swing phase)
    float filletStepPhaseLen;  //!< @brief Nominal size, in terms of approximate phase duration, of the fillets to the leg extension stepping waveform
    float filletPushPhaseLen;  //!< @brief Nominal size, in terms of approximate phase duration, of the fillets to the leg extension pushing waveform
    // ///@}

    // //! @name Halt pose parameters
    // ///@{
    float haltArmExtension;  //!< @brief Halt pose: Extension of the arms (0 = Fully extended, 1 = Fully contracted, see @ref gait::AbstractArmPose "AbstractArmPose")
    float haltArmAngleX;  //!< @brief Halt pose: Roll angle of the arms (positive is away from the body for both arms)
    float haltArmAngleXBias;  //!< @brief Halt pose: Additive anti-symmetric roll angle of the arms (positive is a roll rotation about the positive x axis)
    float haltArmAngleY;      //!< @brief Halt pose: Pitch angle of the central axis of the arms (positive is moving the arms towards the back for both arms)
    float haltLegExtension;  //!< @brief Halt pose: Extension of the legs (0 = Fully extended, 1 = Fully contracted, see @ref gait::AbstractLegPose "AbstractLegPose")
    float haltLegExtensionBias;  //!< @brief Halt pose: Additive one-sided bias of the extension of one of the legs (+ve = Left leg only is shortened by this amount, -ve = Right leg only is shortened by this amount)
    float haltLegAngleX;  //!< @brief Halt pose: Roll angle of the legs (positive is away from the body for both legs)
    float haltLegAngleXBias;  //!< @brief Halt pose: Additive anti-symmetric roll angle of the legs (positive is a roll rotation about the positive x axis)
    float haltLegAngleXNarrow;  //!< @brief Halt pose: Roll angle of the legs (positive is away from the body for both legs) for the narrow feet halt pose
    float haltLegAngleY;   //!< @brief Halt pose: Pitch angle of the central axis of the legs (positive is moving the legs towards the back for both legs)
    float haltLegAngleZ;   //!< @brief Halt pose: Yaw angle of the legs (toe-out is positive for both legs)
    float haltFootAngleX;  //!< @brief Halt pose: Roll angle of the feet relative to the trunk (positive is tilting onto the inner feet for both feet)
    float haltFootAngleXBias;  //!< @brief Halt pose: Additive anti-symmetric roll angle of the feet relative to the trunk (positive is a roll rotation about the positive x axis)
    float haltFootAngleY;  //!< @brief Halt pose: Pitch angle of the feet relative to the trunk (positive is what would make the robot lean back for both feet)
    float haltEffortArm;   //!< @brief Halt pose: Joint effort to use for the arms (in the range `[0,1]`)
    float haltEffortHipYaw;     //!< @brief Halt pose: Joint effort to use for the leg hip yaw (in the range `[0,1]`)
    float haltEffortHipRoll;    //!< @brief Halt pose: Joint effort to use for the leg hip roll (in the range `[0,1]`)
    float haltEffortHipPitch;   //!< @brief Halt pose: Joint effort to use for the leg hip pitch (in the range `[0,1]`)
    float haltEffortKneePitch;  //!< @brief Halt pose: Joint effort to use for the leg knee pitch (in the range `[0,1]`)
    float haltEffortAnklePitch;  //!< @brief Halt pose: Joint effort to use for the leg ankle pitch (in the range `[0,1]`)
    float haltEffortAnkleRoll;  //!< @brief Halt pose: Joint effort to use for the leg ankle roll (in the range `[0,1]`)
    // ///@}

    // //! @name Arm motion parameters
    // ///@{
    float armSagSwingMag;  //!< @brief Magnitude in radians of the arm swing to use at zero biased gait command velocity (for a total peak-to-peak swing of double this value)
    float armSagSwingMagGradX;  //!< @brief Gradient of #armSagSwingMag with respect to the biased gait command x-velocity
    // ///@}

    // //! @name Leg motion parameters
    // ///@{
    float legExtToAngleYGain;  //!< @brief Gain that determines how much the leg extension is also applied to the leg angle Y in order to modify the angle at which the robot lifts its feet, and thereby trim walking on the spot in the sagittal direction
    float legHipAngleXLegExtGain;  //!< @brief Gain that determines how much the appropriate leg is shortened to try to keep the feet level vertically when applying hip angle X
    float legStepHeight;  //!< @brief Nominal swing leg step height (out of the ground, in units of leg extension) to use at zero biased gait command velocity
    float legStepHeightGradX;  //!< @brief Gradient of #legStepHeight with respect to the absolute biased gait command x-velocity
    float legStepHeightGradY;  //!< @brief Gradient of #legStepHeight with respect to the absolute biased gait command y-velocity
    float legPushHeight;  //!< @brief Nominal support leg push height (into the ground, in units of leg extension) to use at zero biased gait command velocity
    float legPushHeightGradX;  //!< @brief Gradient of #legPushHeight with respect to the absolute biased gait command x-velocity
    float legSagSwingMagGradXBwd;  //!< @brief Gradient of the sagittal leg swing magnitude (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is negative (backwards walking)
    float legSagSwingMagGradXFwd;  //!< @brief Gradient of the sagittal leg swing magnitude (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is positive (forwards walking)
    float legSagLeanGradAccXBwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the gait command x-acceleration, when this acceleration is negative (backwards acceleration)
    float legSagLeanGradAccXFwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the gait command x-acceleration, when this acceleration is positive (forwards acceleration)
    float legSagLeanGradVelXBwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is negative (backwards velocity)
    float legSagLeanGradVelXFwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is positive (forwards velocity)
    float legSagLeanGradVelZAbs;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the absolute value of the biased gait command z-velocity
    float legLatSwingMagGradY;    //!< @brief Gradient of the lateral leg swing magnitude (nominally zero radians) with respect to the biased gait command y-velocity
    float legLatHipSwingBias;     //!< @brief Constant bias to the lateral hip swing waveform (hip roll angle, units of radians), used to try to correct for robot walking asymmetries
    float legLatHipSwingMag;  //!< @brief Nominal lateral hip swing magnitude (hip roll angle, units of radians) to use at zero biased gait command velocity
    float legLatHipSwingMagGradX;  //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute biased gait command x-velocity
    float legLatHipSwingMagGradY;  //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute biased gait command y-velocity
    float legLatPushoutMagGradX;   //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command x-velocity
    float legLatPushoutMagGradY;   //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command y-velocity
    float legLatPushoutMagGradZ;   //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command z-velocity
    float legLatLeanGradXZBwd;     //!< @brief Gradient of the lateral lean (nominally zero radians) with respect to the biased gait command x (absolute) and z (signed) velocities, when the x-velocity is
    // //! negative
    float legLatLeanGradXZFwd;  //!< @brief Gradient of the lateral lean (nominally zero radians) with respect to the biased gait command x (absolute) and z (signed) velocities, when the x-velocity is positive
    float legRotSwingMagGradZ;  //!< @brief Gradient of the rotational leg swing magnitude (nominally zero radians) with respect to the biased gait command z-velocity
    float legRotVPushoutMagGradZ;  //!< @brief Gradient of the rotational leg V pushout magnitude (nominally zero radians) with respect to the absolute biased gait command z-velocity
    // ///@}

    // //! @name Tuning parameters
    // ///@{
    bool tuningNoArms;          //!< @brief Disable all gait arm motions // TODO is this important? maybe use when subsumption doesn't have arm control?
    bool tuningNoArmSwing;      //!< @brief Disable all arm swing components
    bool tuningNoArmFeedback;   //!< @brief Disable all arm basic feedback components
    bool tuningNoLegs;          //!< @brief Disable all gait leg motions // TODO is this important?
    bool tuningNoLegLifting;    //!< @brief Disable all leg lifting
    bool tuningNoLegSwing;      //!< @brief Disable all leg swing components
    bool tuningNoLegHipSwing;   //!< @brief Disable all leg hip swing components
    bool tuningNoLegPushout;    //!< @brief Disable all leg pushout components
    bool tuningNoLegLeaning;    //!< @brief Disable all leg leaning components
    bool tuningNoLegVirtual;    //!< @brief Disable all leg virtual slope components
    bool tuningNoLegFeedback;   //!< @brief Disable all leg basic feedback components
    bool tuningNoLegSuppCoeff;  //!< @brief Disable all variations in the leg support coefficients (makes them equal by default instead)
    // ///@}

    // //! @name Basic feedback parameters
    // ///@{
    bool basicGlobalEnable;
    bool basicEnableArmAngleX;
    bool basicEnableArmAngleY;
    bool basicEnableComShiftX;
    bool basicEnableComShiftY;
    bool basicEnableFootAngleCX;
    bool basicEnableFootAngleCY;
    bool basicEnableFootAngleX;
    bool basicEnableFootAngleY;
    bool basicEnableHipAngleX;
    bool basicEnableHipAngleY;
    bool basicEnableTiming;  // Note: To enable basic timing feedback, cmdUseCLTiming must also be true!
    bool basicEnableVirtualSlope;

    float basicFeedBiasArmAngleX;
    float basicFeedBiasArmAngleY;
    float basicFeedBiasComShiftX;
    float basicFeedBiasComShiftY;
    float basicFeedBiasFootAngleX;
    float basicFeedBiasFootAngleY;
    float basicFeedBiasFootAngCX;
    float basicFeedBiasFootAngCY;
    float basicFeedBiasHipAngleX;
    float basicFeedBiasHipAngleY;

    bool basicFusedEnabledLat;
    bool basicFusedEnabledSag;
    int basicFusedFilterN;
    float basicFusedDeadRadiusX;
    float basicFusedDeadRadiusY;
    float basicFusedExpXSinMag;
    float basicFusedExpYSinMag;
    float basicFusedExpXSinOffset;
    float basicFusedExpYSinOffset;
    float basicFusedExpXSinPhase;
    float basicFusedExpYSinPhase;
    float basicFusedGainAllLat;
    float basicFusedGainAllSag;
    float basicFusedArmAngleX;
    float basicFusedArmAngleY;
    float basicFusedComShiftX;
    float basicFusedComShiftY;
    float basicFusedFootAngleX;
    float basicFusedFootAngleY;
    float basicFusedHipAngleX;
    float basicFusedHipAngleY;

    bool basicDFusedEnabledLat;
    bool basicDFusedEnabledSag;
    int basicDFusedFilterN;
    float basicDFusedDeadRadiusX;
    float basicDFusedDeadRadiusY;
    float basicDFusedGainAllLat;
    float basicDFusedGainAllSag;
    float basicDFusedArmAngleX;
    float basicDFusedArmAngleY;
    float basicDFusedComShiftX;
    float basicDFusedComShiftY;
    float basicDFusedFootAngleX;
    float basicDFusedFootAngleY;
    float basicDFusedHipAngleX;
    float basicDFusedHipAngleY;

    bool basicIFusedEnabledLat;
    bool basicIFusedEnabledSag;
    int basicIFusedFilterN;
    float basicIFusedHalfLifeTime;
    float basicIFusedTimeToDecay;
    float basicIFusedTimeToFreeze;
    float basicIFusedGainAllLat;
    float basicIFusedGainAllSag;
    float basicIFusedArmAngleX;
    float basicIFusedArmAngleY;
    float basicIFusedComShiftX;
    float basicIFusedComShiftY;
    float basicIFusedFootAngleCX;
    float basicIFusedFootAngleCY;
    float basicIFusedFootAngleX;
    float basicIFusedFootAngleY;
    float basicIFusedHipAngleX;
    float basicIFusedHipAngleY;

    bool basicGyroEnabledLat;
    bool basicGyroEnabledSag;
    int basicGyroFilterN; // TODO we filter the gyro in our UKF maybe that's good enough
    float basicGyroDeadRadiusX;
    float basicGyroDeadRadiusY;
    float basicGyroExpX;
    float basicGyroExpY;
    float basicGyroGainAllLat;
    float basicGyroGainAllSag;
    float basicGyroArmAngleX;
    float basicGyroArmAngleY;
    float basicGyroComShiftX;
    float basicGyroComShiftY;
    float basicGyroFootAngleX;
    float basicGyroFootAngleY;
    float basicGyroHipAngleX;
    float basicGyroHipAngleY;

    float basicTimingFeedDeadRad;
    float basicTimingGainSlowDown;
    float basicTimingGainSpeedUp;
    float basicTimingWeightFactor;

    float basicComShiftXBuf;
    float basicComShiftXMax;
    float basicComShiftXMin;
    bool basicComShiftXUseLimits;
    float basicComShiftYBuf;
    float basicComShiftYMax;
    float basicComShiftYMin;
    bool basicComShiftYUseLimits;
    float basicFootAnglePhaseLen;  // Determines how quickly the foot angle feedback fades in (and out) after a foot becomes the support foot

    float virtualSlopeOffset;    //!< @brief A constant offset to the virtual slope to continuously apply while walking
    float virtualSlopeGainAsc;   //!< @brief Gradient of the virtual slope with respect to the fused pitch angle (after deadband has been applied) when walking up a virtual slope
    float virtualSlopeGainDsc;   //!< @brief Gradient of the virtual slope with respect to the fused pitch angle (after deadband has been applied) when walking down a virtual slope
    float virtualSlopeMidAngle;  //!< @brief Fused pitch angle for which the (offset-less) virtual slope should be zero
    float virtualSlopeMinAngle;  //!< @brief Minimum radius of the fused pitch angle from virtualSlopeMidAngle before virtual slope starts being applied with a certain gradient (i.e. radius of deadband)
    // ///@}

    // //! @name Capture step parameters
    // ///@{
    bool cmdAllowCLStepSizeX;
    bool cmdUseCLStepSize;  //!< @brief Boolean flag whether computed closed loop step sizes should be used to control the gait, or whether the internal gcv should just be controlled directly from the external gcv input (via maximum gcv acceleration rates)
    bool cmdUseCLTiming;    //!< @brief Boolean flag whether computed closed loop step timing should be used to control the gait, or whether the timing should just remain fixed at the nominal OL frequency
    bool cmdUseNonZeroZMP;  //!< @brief Boolean flag whether the target ZMPs calculated in the LimpModel should be used during walking (i.e. not artificially zeroed right after they are calculated)
    bool cmdUseRXFeedback;  //!< @brief Boolean flag whether the capture gait feedback from RX to MX should be activated (i.e. whether adaptation is used)
    bool cmdUseTXStepSize;  //!< @brief If `cmdUseCLStepSize` is true: Boolean flag whether the closed loop step size calculated by the TX model should be used, or just a fixed nominal CL step size
    bool cmdUseTXTiming;    //!< @brief If `cmdUseCLTiming` is true: Boolean flag whether the closed loop timing calculated by the TX model should be used, or just a fixed nominal CL timing
    float mgC;
    float mgAlpha;
    float mgDelta;
    float mgOmega;
    float mgSigma;
    float mgGamma;
    float mgLatency;
    float mgZmpXMin;
    float mgZmpXMax;
    float mgZmpYMin;
    float mgZmpYMax;
    float mgComOffsetX;
    float mgComOffsetY;
    float mgComOffsetYBias;
    float mgFusedOffsetX;
    float mgFusedOffsetY;
    float mgMaxStepRadiusX;
    float mgMaxStepRadiusY;
    float mgMaxComPositionX;
    float mgPostStepStateCorrAng;
    float nsGain;
    // float nsFusedXRangeLBnd;
    // float nsFusedXRangeUBnd;
    // float nsFusedYRangeLBnd;
    // float nsFusedYRangeUBnd;
    float nsMaxAdaptation;
    float nsAdaptationGain;
    float nsStepNoiseTime;
};


}  // namespace gait

#endif  // GAIT_WALKCONFIG_H
