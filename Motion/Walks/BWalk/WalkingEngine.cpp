/**
* @file WalkingEngine.cpp
* Implementation of a module that creates the walking motions
* @author Colin Graf
*/

#include <cstdio>
#include <assert.h>
#include <cmath>

#include "WalkingEngine.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/General.h"
#include "Kinematics/NUInverseKinematics.h"
#include "Requirements/MassCalibration.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Requirements/ForwardKinematic.h"
#include "NUPlatform/NUPlatform.h"
#include "debug.h"
#include "debugverbositynumotion.h"

//#include "Tools/InverseKinematic.h"
//#include "Tools/MessageQueue/InMessage.h"
//#include "Tools/Streams/InStreams.h"
//#include "Tools/Settings.h"
//#include "Tools/Math/Matrix.h"
//#include "Platform/SoundPlayer.h"

inline float saveAsinh(float xf)
{
  double x = xf; // yes, we need double here
#ifdef _MSC_VER
  return float(log(x + sqrt(x * x + 1.)));
#else
  return float(asinh(x));
#endif
}

inline float saveAcosh(float xf)
{
  //assert(xf >= 1.f);
  double x = xf; // yes, we need double here
  if(x < 1.)
    return 0.000001f;
#ifdef WIN32
  x = log(x + sqrt(x * x - 1.));
#else
  x = acosh(x);
#endif
  if(x < 0.000001)
    return 0.000001f;
  return float(x);
}

template<typename T> ostream& operator<<(ostream& output, const Vector3<T>& v)
{
    output << "[";
    output << v.x << ",";
    output << v.y << ",";
    output << v.z;
    output << "]";
    return output;
}

template<typename T> ostream& operator<<(ostream& output, const Vector2<T>& v)
{
    output << "[";
    output << v.x << ",";
    output << v.y;
    output << "]";
    return output;
}

WalkingEngine::WalkingEngine(NUSensorsData* data, NUActionatorsData* actions, NUInverseKinematics* ik) : NUWalk(data,actions), emergencyShutOff(false), currentMotionType(stand),  m_ik(ik), instable(true), beginOfStable(0)
{
  observedPendulumPlayer.walkingEngine = this;

  // default values for parameters not in the cfg file
  p.standBikeRefX = 20.f;
  p.standStandbyRefX = 3.f;
  //p.standComPosition = Vector3<>(/*20.5f*/ 3.5f /*0.f*/, 50.f, /*259.f*/ /*261.5f*/ 258.0f);
  //p.standComPosition = Vector3<>(3.5f, 50.f,258.0f);
  p.standComPosition = Vector3<>(3.5f, 50.f, 160.0f);
  p.standBodyTilt = 0.03f; //01f;
  p.standArmJointAngles = Vector2<>(0.2f, 0.f);

  p.standHardnessAnklePitch = 75;
  p.standHardnessAnkleRoll = 75;

  //p.walkRefX = 15.f;
//  p.walkRefX = 10.f;
  //p.walkRefXAtFullSpeedX = 7.f;
//  p.walkRefXAtFullSpeedX = 3.f;

//  p.walkRefY = 50.f;
  //p.walkRefY = 40.f;
  //p.walkRefYAtFullSpeedX = 38.f;
//  p.walkRefYAtFullSpeedY = 50.f;
  //p.walkRefYAtFullSpeedY = 40.f;
  //p.walkStepDuration = 470.f;
  //p.walkStepDurationAtFullSpeedX = 400.f;
  //p.walkStepDurationAtFullSpeedY = 400.f;

  p.walkHeight = Vector2<>(p.standComPosition.z,200.f);//OLD VALUEL: p.walkHeight =  = Vector2<>(p.standComPosition.z,300.f);
  p.walkArmRotation = 0.4f;
  p.walkRefXSoftLimit.min = -3.f;
  p.walkRefXSoftLimit.max = 5.f;
  p.walkRefXLimit.min = -15.f;
  p.walkRefXLimit.max = 13.f;
  p.walkRefYLimit.min = -3.f;
  p.walkRefYLimit.max = 3.f;
  p.walkRefYLimitAtFullSpeedX.min = -30.f;
  p.walkRefYLimitAtFullSpeedX.max = 30.f;

  //p.walkLiftOffset = Vector3<>(0.f, -5.f, 17.f);
  //p.walkLiftOffset = Vector3<>(0.f, -5.f, 25.f);
  //p.walkLiftOffsetJerk = 0.f;
  //p.walkLiftOffsetAtFullSpeedY = Vector3<>(0.f, -10.f, 27.f);
  //p.walkLiftRotation = Vector3<>(-0.05f, -0.05f, 0.f);
  //p.walkAntiLiftOffset = Vector3<>(0.f, 0.f, 2.3f);
  //p.walkAntiLiftOffsetAtFullSpeedY = Vector3<>(0.f, 0.f, 2.3f);

  //p.walkComBodyRotation = 0.05f;
  p.walkFadeInShape = Parameters::sine;

  p.kickComPosition = Vector3<>(20.f, 0.f, 245.f);
  p.kickX0Y = 1.f;
  p.kickHeight = Vector2<>(p.standComPosition.z, 300.f);

  //p.speedMax = Pose2D(0.6f, 60.f * 2.f, 50.f);
  //p.speedMaxMin = Pose2D(0.2f, 10.f, 0.f);
  //p.speedMaxBackwards = 50 * 2.f;
  //p.speedMaxChange = Pose2D(0.3f, 8.f, 20.f);

  // Parameters::torsoMatrix Uses the orientation sensor in the model.
  // Parameters::robotModel Uses the forward kinematics in the model.
//  p.observerMeasurementMode = Parameters::torsoMatrix;
  p.observerMeasurementMode = Parameters::robotModel;
  p.observerMeasurementDelay = 40.f;

  p.observerErrorMode = Parameters::mixed;
  p.observerProcessDeviation = Vector4f(0.1f, 0.1f, 3.f, 3.f);
  p.observerMeasurementDeviation = Vector2f(20.f, 20.f);
  p.observerMeasurementDeviationAtFullSpeedX = Vector2f(20.f, 20.f);
  p.observerMeasurementDeviationWhenInstable = Vector2f(20.f, 10.f);

  p.balance = true;
  //p.balance = false;
  p.balanceMinError = Vector3<>(0.f, 0.f, 0.f);
  p.balanceMaxError = Vector3<>(8.f, 8.f, 8.f);
  //p.balanceCom.x = PIDCorrector::Parameters(0.1f, 0.2f, 0.1f, 2.f); // known good value

  p.balanceCom.x = PIDCorrector::Parameters(0.11f, 0.0f, 0.0f, 30.f);
  p.balanceCom.y = PIDCorrector::Parameters(0.11f, 0.0f, 0.0f, 30.f);
  p.balanceCom.z = PIDCorrector::Parameters(0.11f, 0.0f, 0.0f, 30.f);
  p.balanceCom.y = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 0.f);

  p.balanceBodyRotation.x = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 30.f);
  p.balanceBodyRotation.y = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 30.f);
  p.balanceStepSize = Vector2<>(0.08f, -0.04f);
  p.balanceStepSizeWhenInstable = Vector2<>(0.16f, -0.06f);
  p.balanceStepSizeWhenPedantic = Vector2<>(0.04f, -0.02f);
  p.balanceStepSizeInterpolation = 0.1f;

  p.stabilizerOnThreshold = 200.f;
  p.stabilizerOffThreshold = 100.f;
  p.stabilizerDelay = 200;

  p.odometryUseTorsoMatrix = true;
  p.odometryScale = Pose2D(1.f, 1.2f, 1.f);
  p.odometryUpcomingScale = Pose2D(1.f, 1.8f, 1.2f);
  p.odometryUpcomingOffset = Pose2D(0.f, 0.f, 0.f);

  bodyToCom = Vector3<>(0,0,0);
  
  m_walk_parameters.load("BWalk");
  writeParameters();
  
  init();
}

WalkingEngine::~WalkingEngine()
{
    return;
}

void WalkingEngine::init()
{
  // load parameters from config file
    p.computeContants();
#ifdef TARGET_SIM
  p.observerMeasurementDelay = 60.f;
#endif

  currentRefX = p.standStandbyRefX;
  balanceStepSize = p.balanceStepSize;
  m_prev_time = m_data->CurrentTime;
  m_cycle_time = 0;
  m_initialised = false;
  m_walk_requested = false;
  m_pedantic = false;

  float halfArmRotation = p.walkArmRotation * 0.5f;

  // Write initial positions.
  m_initial_larm.resize(m_actions->getSize(NUActionatorsData::LArm),0.0f);
  m_initial_larm[0] = p.standArmJointAngles.x;
  m_initial_larm[1] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y);
  m_initial_larm[2] = -p.standArmJointAngles.y - halfArmRotation;
  if(m_initial_larm.size() > 3)
  {
    m_initial_larm[3] = -mathGeneral::PI/2.0f;
  }

  m_initial_rarm.resize(m_actions->getSize(NUActionatorsData::RArm),0.0f);
  m_initial_rarm[0] = -p.standArmJointAngles.x;
  m_initial_rarm[1] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y);
  m_initial_rarm[2] = -(-p.standArmJointAngles.y - halfArmRotation);
  if(m_initial_larm.size() > 3)
  {
    m_initial_rarm[3] = -(-mathGeneral::PI/2.0f);
  }

  std::vector<float> joints(m_actions->getSize(NUActionatorsData::All),0.0f);

  Matrix leftTarget(4,4,true);

//  leftTarget[0][3] = p.standStandbyRefX+10;
//  leftTarget[1][3] = p.standComPosition.y;
//  leftTarget[2][3] = -p.standComPosition.z + 30.0f;
//  Matrix rightTarget(4,4,true);
//  rightTarget[0][3] = p.standStandbyRefX+10;
//  rightTarget[1][3] = -p.standComPosition.y;
//  rightTarget[2][3] = -p.standComPosition.z + 30.0f;
//  m_ik->calculateLegJoints(leftTarget, rightTarget, joints);

//  std::vector<float>::iterator l_leg_start = joints.begin() + m_actions->getSize(NUActionatorsData::Head) + m_actions->getSize(NUActionatorsData::LArm) + m_actions->getSize(NUActionatorsData::RArm);
//  std::vector<float>::iterator l_leg_end = l_leg_start + m_actions->getSize(NUActionatorsData::LLeg);
//  std::vector<float>::iterator r_leg_start = l_leg_end;
//  std::vector<float>::iterator r_leg_end = r_leg_start + m_actions->getSize(NUActionatorsData::RLeg);

  m_initial_lleg.resize(m_actions->getSize(NUActionatorsData::LLeg),0.f);
  m_initial_lleg[1] = -0.5;
  m_initial_lleg[3] = 1.2;
  m_initial_lleg[5] = -0.7;

  m_initial_rleg = m_initial_lleg;

  nu_nextLeftArmJoints.resize(m_actions->getSize(NUActionatorsData::LArm), 0.0f);   // Left Arm
  nu_nextRightArmJoints.resize(m_actions->getSize(NUActionatorsData::RArm), 0.0f);  // Right Arm
  nu_nextLeftLegJoints.resize(m_actions->getSize(NUActionatorsData::LLeg), 0.0f);   // Left Leg
  nu_nextRightLegJoints.resize(m_actions->getSize(NUActionatorsData::RLeg), 0.0f);  // Right Leg
}

void WalkingEngine::setWalkParameters(const WalkParameters& walkparameters)
{
#if DEBUG_NUMOTION_VERBOSITY > 0
    debug << "WalkingEngine::setWalkParameters: " << std::endl;
#endif
    m_walk_parameters = walkparameters;
    writeParameters();
}

void WalkingEngine::writeParameters()
{
//#if DEBUG_NUMOTION_VERBOSITY > 0
    debug << "WalkingEngine::writeParameters: " << std::endl;
//#endif
    std::vector<Parameter>& params = m_walk_parameters.getParameters();
    for(unsigned int i=0; i<params.size(); i++) {
        std::string& nm = params.at(i).name();
        float value = params.at(i).get();
//#if DEBUG_NUMOTION_VERBOSITY > 0
    debug << nm << " : " << value << std::endl;
//#endif
        if(nm.compare("standComPositionZ") == 0)
            p.standComPosition.z = value;
        else if(nm.compare("walkRefX") == 0)
            p.walkRefX = value;
        else if(nm.compare("walkRefXAtFullSpeedX") == 0)
            p.walkRefXAtFullSpeedX = value;
        else if(nm.compare("walkRefY") == 0)
            p.walkRefY = value;
        else if(nm.compare("walkRefYAtFullSpeedX") == 0)
            p.walkRefYAtFullSpeedX = value;
        else if(nm.compare("walkRefYAtFullSpeedY") == 0)
            p.walkRefYAtFullSpeedY = value;
        else if(nm.compare("walkStepDuration") == 0)
            p.walkStepDuration = value;
        else if(nm.compare("walkStepDurationAtFullSpeedX") == 0)
            p.walkStepDurationAtFullSpeedX = value;
        else if(nm.compare("walkStepDurationAtFullSpeedY") == 0)
            p.walkStepDurationAtFullSpeedY = value;
        else if(nm.compare("walkLiftOffset0") == 0)
            p.walkLiftOffset.x = value;
        else if(nm.compare("walkLiftOffset1") == 0)
            p.walkLiftOffset.y = value;
        else if(nm.compare("walkLiftOffset2") == 0)
            p.walkLiftOffset.z = value;
        else if(nm.compare("walkLiftOffsetJerk") == 0)
            p.walkLiftOffsetJerk = value;
        else if(nm.compare("walkLiftOffsetAtFullSpeedY0") == 0)
            p.walkLiftOffsetAtFullSpeedY.x = value;
        else if(nm.compare("walkLiftOffsetAtFullSpeedY1") == 0)
            p.walkLiftOffsetAtFullSpeedY.y = value;
        else if(nm.compare("walkLiftOffsetAtFullSpeedY2") == 0)
            p.walkLiftOffsetAtFullSpeedY.z = value;
        else if(nm.compare("walkLiftRotation0") == 0)
            p.walkLiftRotation.x = value;
        else if(nm.compare("walkLiftRotation1") == 0)
            p.walkLiftRotation.y = value;
        else if(nm.compare("walkLiftRotation2") == 0)
            p.walkLiftRotation.z = value;
        else if(nm.compare("walkAntiLiftOffset0") == 0)
            p.walkAntiLiftOffset.x = value;
        else if(nm.compare("walkAntiLiftOffset1") == 0)
            p.walkAntiLiftOffset.y = value;
        else if(nm.compare("walkAntiLiftOffset2") == 0)
            p.walkAntiLiftOffset.z = value;
        else if(nm.compare("walkAntiLiftOffsetAtFullSpeedY0") == 0)
            p.walkAntiLiftOffsetAtFullSpeedY.x = value;
        else if(nm.compare("walkAntiLiftOffsetAtFullSpeedY1") == 0)
            p.walkAntiLiftOffsetAtFullSpeedY.y = value;
        else if(nm.compare("walkAntiLiftOffsetAtFullSpeedY2") == 0)
            p.walkAntiLiftOffsetAtFullSpeedY.z = value;
        else if(nm.compare("walkComBodyRotation") == 0)
            p.walkComBodyRotation = value;
        else if(nm.compare("speedMaxRot") == 0)
            p.speedMax.rotation = value;
        else if(nm.compare("speedMaxX") == 0)
            p.speedMax.translation.x = value;
        else if(nm.compare("speedMaxY") == 0)
            p.speedMax.translation.y = value;
        else if(nm.compare("speedMaxMinRot") == 0)
            p.speedMaxMin.rotation = value;
        else if(nm.compare("speedMaxMinX") == 0)
            p.speedMaxMin.translation.x = value;
        else if(nm.compare("speedMaxMinY") == 0)
            p.speedMaxMin.translation.y = value;
        else if(nm.compare("speedMaxBackwards") == 0)
            p.speedMaxBackwards = value;
        else if(nm.compare("speedMaxChangeRot") == 0)
            p.speedMaxChange.rotation = value;
        else if(nm.compare("speedMaxChangeX") == 0)
            p.speedMaxChange.translation.x = value;
        else if(nm.compare("speedMaxChangeY") == 0)
            p.speedMaxChange.translation.y = value;
        else if(nm.compare("balanceComXP") == 0)
            p.balanceCom.x.p = value;
        else if(nm.compare("balanceComXD") == 0)
            p.balanceCom.x.d = value;
        else if(nm.compare("balanceComYP") == 0)
            p.balanceCom.y.p = value;
        else if(nm.compare("balanceComYD") == 0)
            p.balanceCom.y.d = value;
        else if(nm.compare("balanceComZP") == 0)
            p.balanceCom.z.p = value;
        else if(nm.compare("balanceComZD") == 0)
            p.balanceCom.z.d = value;
        else if(nm.compare("balanceBodyRotationXP") == 0)
            p.balanceBodyRotation.x.p = value;
        else if(nm.compare("balanceBodyRotationXD") == 0)
            p.balanceBodyRotation.x.d = value;
        else if(nm.compare("balanceBodyRotationYP") == 0)
            p.balanceBodyRotation.y.p = value;
        else if(nm.compare("balanceBodyRotationYD") == 0)
            p.balanceBodyRotation.y.d = value;
        else
            debug << "WalkingEngine::setWalkParameters(): No matching parameter found: " << nm << std::endl;
    }

    // Copy the default NUwalk parameter speed limits into the B-Human walk speed limits.
    std::vector<float> max_speeds = m_walk_parameters.getMaxSpeeds();
    p.speedMax.translation.x = max_speeds[0] * 10.0f;   // Conversion from cm to mm
    p.speedMax.translation.y = max_speeds[1] * 10.0f;   // Conversion from cm to mm
    p.speedMax.rotation = max_speeds[2];

    //must do this
    p.computeContants();
}

void WalkingEngine::doWalk()
{
    static std::vector<float> joints(m_actions->getSize(NUActionatorsData::All), 0.0f);

//    m_cycle_time = 0.001 * (m_data->CurrentTime - m_prev_time);
//    m_prev_time = m_data->CurrentTime;
    m_cycle_time = 0.02; // time in seconds.
    bool validJoints = m_data->getPosition(NUSensorsData::All, joints);
    if(validJoints)
    {
        theRobotModel.setJointData(joints, theMassCalibration);
        update();
    }
    else
    {
        std::cout << "Joints Invalid" << std::endl;
    }
    return;
}


void WalkingEngine::update(/*WalkingEngineOutput& walkingEngineOutput*/)
{
    bool walking = true;
    bool careful = false;
    m_pedantic = careful;

    float delta_time = m_data->CurrentTime - m_prev_time;
    if(delta_time > (25 * m_cycle_time * 1000))
    {
        m_speed_yaw = 0.0f;
        m_speed_x = 0.0f;
        m_speed_y = 0.0f;
        requestedWalkTarget = Pose2D(0, 0, 0);
        requestedMotionType = stand;
        balanceStepSize = p.balanceStepSize;
    }

    m_prev_time = m_data->CurrentTime;
    
    if(walking/*theMotionSelection.ratios[MotionRequest::walk] > 0.f || theMotionSelection.ratios[MotionRequest::stand] > 0.f*/)
    {
        //Vector2<> targetBalanceStepSize = theMotionRequest.walkRequest.pedantic ? p.balanceStepSizeWhenPedantic : p.balanceStepSize;
        Vector2<> targetBalanceStepSize = m_pedantic ? p.balanceStepSizeWhenPedantic : p.balanceStepSize;
        Vector2<> step = targetBalanceStepSize - balanceStepSize;
        Vector2<> maxStep(std::abs(balanceStepSize.x - p.balanceStepSizeWhenPedantic.x) * p.balanceStepSizeInterpolation,
                          std::abs(balanceStepSize.y - p.balanceStepSizeWhenPedantic.y) * p.balanceStepSizeInterpolation);
        if(step.x > maxStep.x)
            step.x = maxStep.x;
        else if(step.x < -maxStep.x)
            step.x = -maxStep.x;
        if(step.y > maxStep.y)
            step.y = maxStep.y;
        else if(step.y < -maxStep.y)
            step.y = -maxStep.y;
        
        balanceStepSize += step;
        updateMotionRequest();
        updateObservedPendulumPlayer();
        computeMeasuredStance();
        computeExpectedStance();
        computeError();
        updatePendulumPlayer();
        //    updateKickPlayer();
        generateTargetStance();
        generateJointRequest();
        computeOdometryOffset();
        generateOutput(/*walkingEngineOutput*/);
    }
    else
    {
        currentMotionType = stand;
        /*
        if(theMotionSelection.ratios[MotionRequest::specialAction] >= 1.f)
          if(theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead || theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::sitDown)
            currentRefX = p.standStandbyRefX;
            */
        generateDummyOutput(/*walkingEngineOutput*/);
    }
}

void WalkingEngine::updateMotionRequest()
{
//  if(theMotionRequest.motion == MotionRequest::walk)
//  {
//    if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//    {
//      if(theMotionRequest.walkRequest.target != Pose2D())
//        requestedWalkTarget = theMotionRequest.walkRequest.target;
//    }
//    else
//      requestedWalkTarget = theMotionRequest.walkRequest.speed; // just for sgn(requestedWalkTarget.translation.y)
//  }

    // Get values from the NUWalk speeds.
    requestedWalkTarget = Pose2D(m_target_speed_yaw, 10*m_target_speed_x, 10*m_target_speed_y);
//    requestedWalkTarget = Pose2D(1.0, 0, 0);
    requestedMotionType = stand;

    if(fabs(m_target_speed_x) > 0.05 or fabs(m_target_speed_y) > 0.05 or fabs(m_target_speed_yaw) > 0.001)
    {
        m_walk_requested = true;
        if(!instable)
        {
            requestedMotionType = stepping;
        }
    }
    else
    {
        m_walk_requested = false;
    }

//    std::cout << " Requested Motion Type: ";
//    if(requestedMotionType == stepping) std::cout << "Stepping";
//    else if(requestedMotionType == stand) std::cout << "Stand";
//    else std::cout << "Unknown";
//    std::cout << std::endl << "instable = " << (instable?"True":"False") << std::endl;
    
  // get requested motion state
//  requestedMotionType = stand;
//  if((theGroundContactState.contactSafe || !theDamageConfiguration.useGroundContactDetectionForSafeStates) && !theWalkingEngineOutput.enforceStand && theMotionSelection.ratios[MotionRequest::walk] > 0.999f && !instable)
//    if(theMotionRequest.motion == MotionRequest::walk)
//    {
//      if(theMotionRequest.walkRequest.kickType != WalkRequest::none && kickPlayer.isKickStandKick(theMotionRequest.walkRequest.kickType))
//      {
//        bool mirrored = kickPlayer.isKickMirrored(theMotionRequest.walkRequest.kickType);
//        requestedMotionType = mirrored ? standLeft : standRight;
//      }
//      else
//        requestedMotionType = stepping;
//    }
}

void WalkingEngine::updateObservedPendulumPlayer()
{
  // motion update
  if(observedPendulumPlayer.isActive())
    observedPendulumPlayer.seek(m_cycle_time);

  // change motion type
  switch(currentMotionType)
  {
  case stand:
  case standLeft:
  case standRight:
//    if(kickPlayer.isActive())
//      break;

    if(requestedMotionType != currentMotionType)
    {
      SupportLeg supportLeg = SupportLeg(0);
      Vector2<> r;
      Vector2<> x0;
      Vector2<> k = p.walkK;
      StepType stepType = fromStand;
      switch(currentMotionType)
      {
      case standRight:
        assert(false);
        supportLeg = right;
        r = Vector2<>(0.f, -(p.standComPosition.y - p.kickComPosition.y + p.kickX0Y));
        x0 = Vector2<>(currentRefX, p.kickX0Y);
        k = p.kickK;
        stepType = fromStandLeft;
        break;
      case standLeft:
        assert(false);
        supportLeg = left;
        r = Vector2<>(0.f, p.standComPosition.y - p.kickComPosition.y + p.kickX0Y);
        x0 = Vector2<>(currentRefX, -p.kickX0Y);
        k = p.kickK;
        stepType = fromStandRight;
        break;
      case stand:
        if(requestedMotionType == standRight || (requestedMotionType == stepping && requestedWalkTarget.translation.y > 0.f))
        {
          supportLeg = left;
          r = Vector2<>(currentRefX, p.walkRefY);
          x0 = Vector2<>(0.f, -p.walkRefY);
        }
        else
        {
          supportLeg = right;
          r = Vector2<>(currentRefX, -p.walkRefY);
          x0 = Vector2<>(0.f, p.walkRefY);
        }
        break;
      default:
        assert(false);
        break;
      }
      lastNextSupportLeg = supportLeg;
      lastSelectedSpeed = Pose2D();
      nextPendulumParameters.s = StepSize();
      observedPendulumPlayer.init(stepType, p.observerMeasurementDelay * -0.001f, supportLeg, r, x0, k, m_cycle_time);
      currentMotionType = stepping;
    }
    break;
  default:
    break;
  }
}

void WalkingEngine::computeMeasuredStance()
{
    float heightLeg5Joint = 45.19;//old value for NAO heightLeg5Joint = 45.19; Should be 33.5! But when set that way walking ceases.
    std::vector<float> orientation_sensors;
    m_data->getOrientation(orientation_sensors);
    const Vector3<> axis(orientation_sensors[0], orientation_sensors[1], 0);
    RotationMatrix torso(axis);

  switch(p.observerMeasurementMode)
  {
  case Parameters::robotModel:  // This method uses the kinematic model
  {

    Pose3D comToLeft = Pose3D(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footLeft]).translate(0.f, 0.f, -heightLeg5Joint);
    Pose3D comToRight = Pose3D(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footRight]).translate(0.f, 0.f, -heightLeg5Joint);
    Pose3D leftToCom = comToLeft.invert();
    Pose3D rightToCom = comToRight.invert();

    RotationMatrix rotationLeft(Vector3<>(
                                  atan2(leftToCom.rotation.c1.z, leftToCom.rotation.c2.z),
                                  atan2(-leftToCom.rotation.c0.z, leftToCom.rotation.c2.z),
                                  0.f));
    RotationMatrix rotationRight(Vector3<>(
                                   atan2(rightToCom.rotation.c1.z, rightToCom.rotation.c2.z),
                                   atan2(-rightToCom.rotation.c0.z, rightToCom.rotation.c2.z),
                                   0.f));

    RotationMatrix& bodyRotation = observedPendulumPlayer.supportLeg == left ? rotationLeft : rotationRight;
    // TODO: optimize

    measuredLeftToCom = -Pose3D(bodyRotation).conc(comToLeft).translation;
    measuredRightToCom = -Pose3D(bodyRotation).conc(comToRight).translation;
  }
  break;

  default:  // This method uses the body orientation
    measuredLeftToCom = -Pose3D(torso).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footLeft]).translate(0.f, 0.f, -heightLeg5Joint).translation;
    measuredRightToCom = -Pose3D(torso).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footRight]).translate(0.f, 0.f, -heightLeg5Joint).translation;
    break;
  }
}

void WalkingEngine::computeExpectedStance()
{
  LegStance* expectedStance;
  if(legStances.getNumberOfEntries() == 0)
  {
    legStances.add();
    expectedStance = &legStances.getEntry(0);
    getStandStance(*expectedStance);
    stepOffset = StepSize();
  }
  else
  {
    int index = std::min(int(p.observerMeasurementDelay / 10.f - 0.5f), legStances.getNumberOfEntries() - 1);
    expectedStance = &legStances.getEntry(index);
    if(observedPendulumPlayer.isActive() && !observedPendulumPlayer.isLaunching())
      observedPendulumPlayer.getStance(*expectedStance, 0, 0, &stepOffset);
    else
    {
      stepOffset = StepSize();
    }
  }

  expectedLeftToCom = expectedStance->leftOriginToCom - expectedStance->leftOriginToFoot.translation;
  expectedRightToCom = expectedStance->rightOriginToCom - expectedStance->rightOriginToFoot.translation;
}

void WalkingEngine::computeError()
{
    bool fallen = m_data->isFallen();
    bool falling = m_data->isFalling();

//  if((theGroundContactState.contactSafe || !theDamageConfiguration.useGroundContactDetectionForSafeStates) && theFallDownState.state != FallDownState::onGround && theFallDownState.state != FallDownState::falling)
  if(!fallen && !falling)
  {
    leftError = measuredLeftToCom - expectedLeftToCom;
    rightError = measuredRightToCom - expectedRightToCom;

//    std::cout << "leftError: " << leftError << "; measuredLeftToCom: " << measuredLeftToCom << "; expectedLeftToCom: " << expectedLeftToCom << std::endl;
//    std::cout << "rightError: " << rightError << "; measuredRightToCom: " << measuredRightToCom << "; expectedRightToCom: " << expectedRightToCom << std::endl;

    // I'm not sure what this part does
//    if(theMotionSelection.ratios[MotionRequest::walk] < 0.99f)
//    {
//      instability.add(((leftError + rightError) * 0.5f).squareAbs());
//      instable = true;
//    }
//    else if(kickPlayer.isActive())
//    {
//      instability.add((observedPendulumPlayer.supportLeg == left ? leftError : rightError).squareAbs());
//    }
//    else
//    {
      instability.add(((leftError + rightError) * 0.5f).squareAbs());
      if(instability.getAverage() > p.stabilizerOnThreshold)
      {
        instable = true;
        beginOfStable = 0;
      }
      else if(instability.getAverage() < p.stabilizerOffThreshold)
      {
        if(!beginOfStable)
          beginOfStable = m_data->CurrentTime;
        else if((m_data->CurrentTime - beginOfStable) > p.stabilizerDelay)
          instable = false;
      }
  }
    for(int i = 0; i < 2; ++i)
    {
      Vector3<>& error = i == 0 ? leftError : rightError;
      for(int i = 0; i < 3; ++i)
      {
        if(error[i] > -p.balanceMinError[i] && error[i] < p.balanceMinError[i])
          error[i] = 0.f;
        else if(error[i] > p.balanceMinError[i])
          error[i] -= p.balanceMinError[i];
        else
          error[i] += p.balanceMinError[i];

        if(error[i] > p.balanceMaxError[i])
          error[i] = p.balanceMaxError[i];
        else if(error[i] < -p.balanceMaxError[i])
          error[i] = -p.balanceMaxError[i];
      }
    }
}

void WalkingEngine::updatePendulumPlayer()
{
  if(currentMotionType == stepping)
  {
    if(p.balance)
      observedPendulumPlayer.applyCorrection(leftError, rightError, m_cycle_time);

    pendulumPlayer = observedPendulumPlayer;
    pendulumPlayer.seek(p.observerMeasurementDelay * 0.001f);

    if(!pendulumPlayer.isActive())
    {
      currentRefX = pendulumPlayer.next.r.x;
      switch(pendulumPlayer.type)
      {
      case toStand:
        currentMotionType = stand;
        break;
      case toStandLeft:
        currentMotionType = standLeft;
        break;
      case toStandRight:
        currentMotionType = standRight;
        break;
      default:
        break;
      }
      //if(currentMotionType == requestedMotionType && (requestedMotionType == standLeft || requestedMotionType == standRight) && theMotionRequest.walkRequest.kickType != WalkRequest::none)
      //kickPlayer.init(theMotionRequest.walkRequest.kickType, theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
    }
  }
}

//void WalkingEngine::updateKickPlayer()
//{
//  if(currentMotionType == stepping)
//  {
//    if(!kickPlayer.isActive() && pendulumPlayer.kickType != WalkRequest::none)
//    {
//      kickPlayer.init(pendulumPlayer.kickType, theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
//    }
//    if(kickPlayer.isActive())
//    {
//      if(kickPlayer.getType() != pendulumPlayer.kickType)
//        kickPlayer.stop();
//      else
//      {
//        float length = kickPlayer.getLength();
//        assert(length >= 0.f);
//        float pos = length * (pendulumPlayer.t - pendulumPlayer.tb) / (pendulumPlayer.te - pendulumPlayer.tb);
//        kickPlayer.seek(std::max(pos - kickPlayer.getCurrentPosition(), 0.f));
//      }
//    }
//  }
//  else
//  {
//    if(kickPlayer.isActive())
//      kickPlayer.seek(theFrameInfo.cycleTime);
//    else if(theMotionRequest.walkRequest.kickType != WalkRequest::none && currentMotionType == requestedMotionType && (requestedMotionType == standLeft || requestedMotionType == standRight))
//      kickPlayer.init(theMotionRequest.walkRequest.kickType, theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
//  }
//}

void WalkingEngine::generateTargetStance()
{
  std::vector<float> headPos;
  if(m_data->getPosition(NUSensorsData::Head,headPos))
  {
      targetStance.headJointAngles[0] = headPos[0];//theHeadJointRequest.pan;
      targetStance.headJointAngles[1] = headPos[1];//theHeadJointRequest.tilt;
  }

  float leftArmAngle = 0.f, rightArmAngle = 0.f;
  if(currentMotionType == stepping)
    pendulumPlayer.getStance(targetStance, &leftArmAngle, &rightArmAngle, 0);
  else
    getStandStance(targetStance);

  // set arm angles
  float halfArmRotation = p.walkArmRotation * 0.5f;
  targetStance.leftArmJointAngles[0] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y + leftArmAngle);
  targetStance.leftArmJointAngles[1] = p.standArmJointAngles.x;
  targetStance.leftArmJointAngles[2] = -mathGeneral::PI/2.0f;
  targetStance.leftArmJointAngles[3] = -p.standArmJointAngles.y - leftArmAngle - halfArmRotation;
  targetStance.rightArmJointAngles[0] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y + rightArmAngle);
  targetStance.rightArmJointAngles[1] = -(p.standArmJointAngles.x);
  targetStance.rightArmJointAngles[2] = -(-mathGeneral::PI/2.0f);
  targetStance.rightArmJointAngles[3] = -(-p.standArmJointAngles.y - rightArmAngle - halfArmRotation);

  // playing a kick motion!?
//  if(kickPlayer.isActive())
//  {
//    kickPlayer.setParameters(theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
//    kickPlayer.apply(targetStance);
//  }

  legStances.add(targetStance);
}

void WalkingEngine::getStandStance(LegStance& stance) const
{
  stance.leftOriginToFoot = Pose3D(Vector3<>(0.f, p.standComPosition.y, 0.f));
  stance.rightOriginToFoot = Pose3D(Vector3<>(0.f, -p.standComPosition.y, 0.f));
  if(currentMotionType == stand)
  {
    stance.leftOriginToCom = stance.rightOriginToCom = Vector3<>(p.standComPosition.x + currentRefX, 0.f, p.standComPosition.z);
    //stance.leftOriginToCom = stance.rightOriginToCom = Vector3<>(p.standComPosition.x + p.standStandbyRefX, 0.f, p.standComPosition.z);
  }
  else
  {
    const float sign = currentMotionType == standLeft ? 1.f : -1.f;
    stance.leftOriginToCom =  stance.rightOriginToCom = Vector3<>(p.kickComPosition.x + currentRefX, (p.standComPosition.y - p.kickComPosition.y) * sign, p.kickComPosition.z);
  }
}

void WalkingEngine::generateJointRequest()
{
  Vector3<> correctedLeftOriginToCom = targetStance.leftOriginToCom;
  Vector3<> correctedRightOriginToCom = targetStance.rightOriginToCom;

  if(p.balance)
  {
    correctedLeftOriginToCom.x += leftControllerX.getCorrection(leftError.x, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.x);
    correctedLeftOriginToCom.y += leftControllerY.getCorrection(leftError.y, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.y);
    correctedLeftOriginToCom.z += leftControllerZ.getCorrection(leftError.z, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.z);
    correctedRightOriginToCom.x += rightControllerX.getCorrection(rightError.x, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.x);
    correctedRightOriginToCom.y += rightControllerY.getCorrection(rightError.y, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.y);
    correctedRightOriginToCom.z += rightControllerZ.getCorrection(rightError.z, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.z);
  }

  if(currentMotionType == stepping)
  {
    if(pendulumPlayer.supportLeg == left)
    {
      if(pendulumPlayer.l.z != 0.f)
        correctedRightOriginToCom.z -= p.walkLiftOffsetJerk;
    }
    else
    {
      if(pendulumPlayer.l.z != 0.f)
        correctedLeftOriginToCom.z -= p.walkLiftOffsetJerk;
    }
  }


  const float heightLeg5Joint = 45.19;

  std::vector<float> joint_positions;
  m_data->getPosition(NUSensorsData::All, joint_positions);
  unsigned int total_arm_joints = m_actions->getSize(NUActionatorsData::LArm);

  std::vector<float>::iterator l_arm_it = joint_positions.begin() + m_actions->getSize(NUActionatorsData::Head);
  std::vector<float>::iterator l_arm_end = l_arm_it + m_actions->getSize(NUActionatorsData::LArm);
  std::vector<float>::iterator r_arm_it = l_arm_end;
  std::vector<float>::iterator r_arm_end = r_arm_it + m_actions->getSize(NUActionatorsData::RArm);

  *(l_arm_it++) = targetStance.leftArmJointAngles[1];
  *(l_arm_it++) = targetStance.leftArmJointAngles[0];
  *(l_arm_it++) = targetStance.leftArmJointAngles[3];

  if(total_arm_joints > 3)
  {
    *(l_arm_it++) = targetStance.leftArmJointAngles[2];
  }

  assert(l_arm_it == l_arm_end);

  *(r_arm_it++) = targetStance.rightArmJointAngles[1];
  *(r_arm_it++) = targetStance.rightArmJointAngles[0];
  *(r_arm_it++) = -targetStance.rightArmJointAngles[3];
  if(total_arm_joints > 3)
  {
      *(r_arm_it-1) = -*(r_arm_it-1);
      *(r_arm_it++) = targetStance.rightArmJointAngles[2];
  }

  assert(r_arm_it == r_arm_end);

  float bodyRotationX = 0.f, bodyRotationY = 0.f;
  if(p.balance)
  {
    bodyRotationY = atan(bodyControllerX.getCorrection((leftError.x + rightError.x) * 0.5f, m_cycle_time, p.observerMeasurementDelay, p.balanceBodyRotation.x) / p.walkHeight.x);
    bodyRotationX = atan(bodyControllerY.getCorrection((leftError.y + rightError.y) * 0.5f, m_cycle_time, p.observerMeasurementDelay, p.balanceBodyRotation.y) / p.walkHeight.y);
  }

  float additionalBodyRotation = (((targetStance.rightOriginToCom.y - targetStance.rightOriginToFoot.translation.y) - p.standComPosition.y) + ((targetStance.leftOriginToCom.y - targetStance.leftOriginToFoot.translation.y) + p.standComPosition.y)) * 0.5f;
  additionalBodyRotation *= 1.f / (22.5f - 50.f);
  additionalBodyRotation *= p.walkComBodyRotation;
  RotationMatrix bodyRotation(Vector3<>(additionalBodyRotation + bodyRotationX, bodyRotationY, 0.f));
  bodyRotation *= p.standBodyRotation;

  const Pose3D comToLeftOrigin = Pose3D(bodyRotation, correctedLeftOriginToCom).invert();
  const Pose3D comToRightOrigin = Pose3D(bodyRotation, correctedRightOriginToCom).invert();

  // TODO: optimize this by calculating the inverted left/rightOriginToCom pose directly
  const Pose3D comToLeftAnkle = Pose3D(comToLeftOrigin).conc(targetStance.leftOriginToFoot).translate(0.f, 0.f, heightLeg5Joint);
  const Pose3D comToRightAnkle = Pose3D(comToRightOrigin).conc(targetStance.rightOriginToFoot).translate(0.f, 0.f, heightLeg5Joint);
  const Vector3<> averageComToAnkle = (comToLeftAnkle.translation + comToRightAnkle.translation) * 0.5f;

  Vector3<> bodyToCom = this->bodyToCom;
  Vector3<> bodyToComOffset = lastAverageComToAnkle != Vector3<>() ? (averageComToAnkle - lastAverageComToAnkle) * 0.4f : Vector3<>();
  lastAverageComToAnkle = averageComToAnkle;
  bodyToCom += bodyToComOffset;

  Pose3D bodyToLeftAnkle(comToLeftAnkle.rotation, bodyToCom + comToLeftAnkle.translation);

  Pose3D bodyToRightAnkle(comToRightAnkle.rotation, bodyToCom + comToRightAnkle.translation);
  bool reachable = m_ik->calculateLegJoints(Pose2Matrix(bodyToLeftAnkle), Pose2Matrix(bodyToRightAnkle), joint_positions);

  for(int i = 0; i < 7; ++i)
  {
    if(reachable || this->bodyToCom == Vector3<>())
    {
      if(reachable)
      {
        this->bodyToCom = bodyToCom; // store the working bodyToCom offset
      }

      RobotModel robotModel(joint_positions, theMassCalibration);

      // TODO: improve this by not calculating the whole limb/mass model in each iteration

      Pose3D tmpComToLeftAnkle = Pose3D(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibration::footLeft]);
      Pose3D tmpComToRightAnkle = Pose3D(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibration::footRight]);
      tmpComToLeftAnkle.rotation = RotationMatrix();
      tmpComToRightAnkle.rotation = RotationMatrix();

      Vector3<> tmpAverageComToAnkle = (tmpComToLeftAnkle.translation + tmpComToRightAnkle.translation) * 0.5f;
      bodyToComOffset = (averageComToAnkle - tmpAverageComToAnkle) * 1.3f;
    }
    else
    {
      bodyToCom = this->bodyToCom; // recover last working bodyToCom offset
      bodyToComOffset *= 0.5f; // reduce last bodyToComOffset
    }

    bodyToCom += bodyToComOffset;

    bodyToLeftAnkle.translation = bodyToCom + comToLeftAnkle.translation;
    bodyToRightAnkle.translation = bodyToCom + comToRightAnkle.translation;

    reachable = m_ik->calculateLegJoints(Pose2Matrix(bodyToLeftAnkle), Pose2Matrix(bodyToRightAnkle), joint_positions);
    RobotModel robotModel(joint_positions, theMassCalibration);

    if(abs(bodyToComOffset.x) < 0.05 && abs(bodyToComOffset.y) < 0.05 && abs(bodyToComOffset.z) < 0.05)
    {
      break;
    }
  }

  std::vector<float>::iterator startIndex = joint_positions.begin()+2;
  std::vector<float>::iterator endIndex = startIndex + nu_nextLeftArmJoints.size();
  nu_nextLeftArmJoints.assign(startIndex, endIndex);
  startIndex = endIndex;
  endIndex = startIndex + nu_nextRightArmJoints.size();
  nu_nextRightArmJoints.assign(startIndex, endIndex);
  startIndex = endIndex;
  endIndex = startIndex + nu_nextLeftLegJoints.size();
  nu_nextLeftLegJoints.assign(startIndex, endIndex);
  startIndex = endIndex;
  endIndex = startIndex + nu_nextRightLegJoints.size();
  nu_nextRightLegJoints.assign(startIndex, endIndex);
  
  // Hack to move both feet.
  const unsigned int hip_yaw_index = 2;
  if (fabs(nu_nextRightLegJoints[hip_yaw_index]) < fabs(nu_nextLeftLegJoints[hip_yaw_index]))
  {
    nu_nextRightLegJoints[hip_yaw_index] = nu_nextLeftLegJoints[hip_yaw_index] / -2.0f;
    nu_nextLeftLegJoints[hip_yaw_index] /= 2.0f;
  }
  else if (fabs(nu_nextRightLegJoints[hip_yaw_index]) >  fabs(nu_nextLeftLegJoints[hip_yaw_index]))
  {
    nu_nextLeftLegJoints[hip_yaw_index] = nu_nextRightLegJoints[hip_yaw_index] / -2.0f;
    nu_nextRightLegJoints[hip_yaw_index] /= 2.0f;
  }

  // If generated motion was a standing motion, save this as the new initial target.
  if(currentMotionType == stand)
  {
      m_initial_lleg.assign(nu_nextLeftLegJoints.begin(), nu_nextLeftLegJoints.end());
      m_initial_rleg.assign(nu_nextRightLegJoints.begin(), nu_nextRightLegJoints.end());
  }

    return;
}

void WalkingEngine::generateOutput(/*WalkingEngineOutput& walkingEngineOutput*/)
{
//  if(observedPendulumPlayer.isActive())
//  {
//    const float stepDuration = (observedPendulumPlayer.te - observedPendulumPlayer.next.tb) * 2.f;
//    walkingEngineOutput.speed.translation = Vector2<>(observedPendulumPlayer.s.translation.x + observedPendulumPlayer.next.s.translation.x, observedPendulumPlayer.s.translation.y + observedPendulumPlayer.next.s.translation.y) / stepDuration;
//    walkingEngineOutput.speed.rotation = (observedPendulumPlayer.s.rotation + observedPendulumPlayer.next.s.rotation) / stepDuration;
//  }
//  else
//    walkingEngineOutput.speed = Pose2D();

//  walkingEngineOutput.odometryOffset = odometryOffset;
//  walkingEngineOutput.upcomingOdometryOffset = upcomingOdometryOffset;
//  walkingEngineOutput.upcomingOdometryOffsetValid = upcomingOdometryOffsetValid;
//  walkingEngineOutput.isLeavingPossible = currentMotionType == stand;
//  if(currentMotionType == stepping)
//    walkingEngineOutput.positionInWalkCycle = 0.5f * ((observedPendulumPlayer.t - observedPendulumPlayer.tb) / (observedPendulumPlayer.te - observedPendulumPlayer.tb)) + (observedPendulumPlayer.supportLeg == left ? 0.5f : 0.f);
//  else
//    walkingEngineOutput.positionInWalkCycle = 0.f;
//  walkingEngineOutput.enforceStand = false;
//  walkingEngineOutput.instability = 0.f;
//  walkingEngineOutput.executedWalk = theMotionRequest.walkRequest;
//  walkingEngineOutput.executedWalk.kickType = kickPlayer.isActive() ? kickPlayer.getType() : WalkRequest::none;
//  (JointRequest&)walkingEngineOutput = jointRequest;

    float default_arm_stifness = 30.0f;
    // Set the arm positions.
    m_actions->add(NUActionatorsData::RArm, Platform->getTime(), nu_nextRightArmJoints, default_arm_stifness);
    m_actions->add(NUActionatorsData::LArm, Platform->getTime(), nu_nextLeftArmJoints, default_arm_stifness);

    float default_leg_stifness = 75.0f;

    // Reduce stiffness if we are standing.
    if(currentMotionType == stand)
    {
        default_leg_stifness = 75.0f;
    }

    // Set the leg positions.
    std::vector<float> legstiffness(m_actions->getSize(NUActionatorsData::LLeg), default_leg_stifness);
    legstiffness[5] = p.standHardnessAnkleRoll;
    legstiffness[6] = p.standHardnessAnklePitch;

    m_actions->add(NUActionatorsData::RLeg, 0, nu_nextRightLegJoints, legstiffness);
    m_actions->add(NUActionatorsData::LLeg, 0, nu_nextLeftLegJoints, legstiffness);
    return;
}

void WalkingEngine::generateDummyOutput(/*WalkingEngineOutput& walkingEngineOutput*/)
{
    return;
//  walkingEngineOutput.speed = Pose2D();
//  walkingEngineOutput.odometryOffset = Pose2D();
//  walkingEngineOutput.upcomingOdometryOffset = Pose2D();
//  walkingEngineOutput.upcomingOdometryOffsetValid = true;
//  walkingEngineOutput.isLeavingPossible = true;
//  walkingEngineOutput.positionInWalkCycle = 0.f;
//  walkingEngineOutput.enforceStand = false;
//  walkingEngineOutput.instability = 0.f;
//  walkingEngineOutput.executedWalk = WalkRequest();
  // leaving joint data untouched
}

void WalkingEngine::generateNextStepSize(SupportLeg nextSupportLeg, StepType lastStepType, /*WalkRequest::KickType lastKickType,*/ PendulumParameters& next)
{
  if(nextSupportLeg == lastNextSupportLeg)
    next = nextPendulumParameters;
  else
  {
    lastNextSupportLeg = nextSupportLeg;

    StepSize lastStepSize = next.s;

    const float sign = nextSupportLeg == right ? 1.f : -1.f;
    next.type = unknown;
    next.s = StepSize();
    next.l = Vector3<>(p.walkLiftOffset.x, p.walkLiftOffset.y * sign, p.walkLiftOffset.z);
    next.al = Vector3<>(p.walkAntiLiftOffset.x, p.walkAntiLiftOffset.y * sign, p.walkAntiLiftOffset.z);
    next.lRotation = Vector3<>();
    next.r = Vector2<>(p.walkRefX, p.walkRefY * (-sign));
    next.c = Vector2<>();
    next.k = p.walkK;
    next.te = p.te;
    next.tb = -p.te;
//    next.kickType = WalkRequest::none;
    next.sXLimit.max = p.speedMax.translation.x * (1.1f * 0.5f);
    next.sXLimit.min = p.speedMaxBackwards * (-1.1f * 0.5f);
    next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
    next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
    next.rYLimit.max = p.walkRefY + p.walkRefYLimit.max;
    next.rYLimit.min = p.walkRefY + p.walkRefYLimit.min;

    switch(lastStepType)
    {
    case toStand:
      next.te = next.tb = 0.f;
//      if(theMotionRequest.motion == MotionRequest::bike)
//        next.r.x  = p.standBikeRefX;
//      else
        next.r.x = p.walkRefX;
      next.x0 = Vector2<>(0.f, -next.r.y);
      next.xv0 = Vector2<>();
      next.xtb = Vector2<>(next.r.x, 0.f);
      next.xvtb = next.xv0;
      break;
    case toStandLeft:
    case toStandRight:
      assert(false); // TODO!
      break;

    default:

      switch(requestedMotionType)
      {
      case stand:
//        if(theMotionRequest.motion == MotionRequest::bike && (nextSupportLeg == left) == theMotionRequest.bikeRequest.mirror)
//          break;
        if(abs(lastStepSize.translation.x) > p.speedMax.translation.x * 0.5f)
          break;
        next.type = toStand;
        next.te = p.te;
        next.tb = -p.te;
        break;

      case standLeft:
      case standRight:
        if((nextSupportLeg == left && requestedMotionType == standLeft) || (nextSupportLeg == right && requestedMotionType == standRight))
        {
          assert(false); // TODO!
          next.type = requestedMotionType == standLeft ? toStandLeft : toStandRight;
          //next.r = Vector2<>(0.f, (p.standComPosition.y - p.kickComPosition.y + p.kickX0Y) * (-sign));
          //next.x0 = Vector2<>(0.f, p.kickX0Y * sign);
          //next.k = p.kickK;
        }
        break;
      default:
        break;
      }
      if(next.type == unknown)
      {
//        if(!instable && theMotionRequest.walkRequest.kickType != WalkRequest::none && !kickPlayer.isKickStandKick(theMotionRequest.walkRequest.kickType) &&
//           kickPlayer.isKickMirrored(theMotionRequest.walkRequest.kickType) == (nextSupportLeg == left) &&
//           theMotionRequest.walkRequest.kickType != lastExecutedWalkingKick)
//        {
//          lastExecutedWalkingKick = theMotionRequest.walkRequest.kickType;
//          next.kickType = theMotionRequest.walkRequest.kickType;
//          kickPlayer.getKickPreStepSize(next.kickType, next.s.rotation, next.s.translation);
//          next.r.x = kickPlayer.getKickRefX(next.kickType, next.r.x);
//          next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
//          next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
//          next.rYLimit.max = p.walkRefY + p.walkRefYLimitAtFullSpeedX.max;
//          next.rYLimit.min = p.walkRefY + p.walkRefYLimitAtFullSpeedX.min;
//          float duration = kickPlayer.getKickDuration(next.kickType);
//          if(duration != 0.f)
//          {
//            next.te = duration * 0.25f;
//            next.tb = -next.te;
//          }
//        }
        /*
        else if(lastKickType != WalkRequest::none)
        {
          kickPlayer.getKickStepSize(lastKickType, next.s.rotation, next.s.translation);
          next.r.x = kickPlayer.getKickRefX(lastKickType, next.r.x);
          next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
          next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
          next.rYLimit.max = p.walkRefY + p.walkRefYLimitAtFullSpeedX.max;
          next.rYLimit.min = p.walkRefY + p.walkRefYLimitAtFullSpeedX.min;
          float duration = kickPlayer.getKickDuration(lastKickType);
          if(duration != 0.f)
          {
            next.te = duration * 0.25f;
            next.tb = -next.te;
          }
        }

        else*/ if(instable)
        {
          // nothing
        }
        else
        {
//          if(theMotionRequest.walkRequest.kickType == WalkRequest::none)
//            lastExecutedWalkingKick = WalkRequest::none;

          // get requested walk target and speed
          Pose2D walkTarget = requestedWalkTarget;
          //Pose2D requestedSpeed = theMotionRequest.walkRequest.speed;
          Pose2D requestedSpeed = Pose2D(m_speed_yaw, 10*m_speed_x, 10*m_speed_y);
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode) // remove upcoming odometry offset
//          {

//            Pose2D upcomingOdometryOffset = observedPendulumPlayer.next.s - stepOffset * 0.5f; // == (observedPendulumPlayer.next.s - stepOffset) * 0.5f + observedPendulumPlayer.next.s * 0.5f
//            //upcomingOdometryOffset -= observedPendulumPlayer.s * 0.5f;

//            upcomingOdometryOffset.translation.x *= p.odometryUpcomingScale.translation.x;
//            upcomingOdometryOffset.translation.y *= p.odometryUpcomingScale.translation.y;
//            upcomingOdometryOffset.rotation *= p.odometryUpcomingScale.rotation;

//            float sign = observedPendulumPlayer.supportLeg == left ? -1.f : 1.f;
//            Pose2D up(p.odometryUpcomingOffset.rotation * sign, p.odometryUpcomingOffset.translation.x, p.odometryUpcomingOffset.translation.y * sign);
//            upcomingOdometryOffset += up;

//            walkTarget -= upcomingOdometryOffset;
//            walkTarget -= up;
//            requestedSpeed = Pose2D(walkTarget.rotation * 2.f / p.odometryUpcomingScale.rotation, walkTarget.translation.x * 2.f / p.odometryUpcomingScale.translation.x, walkTarget.translation.y * 2.f / p.odometryUpcomingScale.translation.y);

//            // x-speed clipping to handle limited deceleration
//            //float maxSpeedForTargetX = sqrt(2.f * abs(requestedSpeed.translation.x) * p.speedMaxChangeX);
//            //if(abs(requestedSpeed.translation.x) > maxSpeedForTargetX)
//            //requestedSpeed.translation.x = requestedSpeed.translation.x >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;
//          }
//          else if(theMotionRequest.walkRequest.mode == WalkRequest::percentageSpeedMode)
//          {
//            requestedSpeed.rotation *= p.speedMax.rotation;
//            requestedSpeed.translation.x *= (requestedSpeed.translation.x >= 0.f ? p.speedMax.translation.x : p.speedMaxBackwards);
//            requestedSpeed.translation.y *= p.speedMax.translation.y;
//          }

          // compute max speeds for the requested walk direction
          Pose2D maxSpeed(p.speedMax.rotation, requestedSpeed.translation.x < 0.f ? p.speedMaxBackwards : p.speedMax.translation.x, p.speedMax.translation.y);
          Vector3<> tmpSpeed(
            requestedSpeed.translation.x / (p.speedMaxMin.translation.x + maxSpeed.translation.x),
            requestedSpeed.translation.y / (p.speedMaxMin.translation.y + maxSpeed.translation.y),
            requestedSpeed.rotation / (p.speedMaxMin.rotation + maxSpeed.rotation));
          const float tmpSpeedAbs = tmpSpeed.abs();
          if(tmpSpeedAbs > 1.f)
          {
            tmpSpeed /= tmpSpeedAbs;
            tmpSpeed.x *= (p.speedMaxMin.translation.x + maxSpeed.translation.x);
            tmpSpeed.y *= (p.speedMaxMin.translation.y + maxSpeed.translation.y);
            tmpSpeed.z *= (p.speedMaxMin.rotation + maxSpeed.rotation);
            maxSpeed.translation.x = std::min(std::abs(tmpSpeed.x), maxSpeed.translation.x);
            maxSpeed.translation.y = std::min(std::abs(tmpSpeed.y), maxSpeed.translation.y);
            maxSpeed.rotation = std::min(std::abs(tmpSpeed.z), maxSpeed.rotation);
          }

//          // x-speed clipping to handle limited deceleration
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//          {
//            float maxSpeedForTargetX = sqrt(2.f * abs(requestedSpeed.translation.x) * p.speedMaxChange.translation.x);
//            if(abs(requestedSpeed.translation.x) > maxSpeedForTargetX)
//              requestedSpeed.translation.x = requestedSpeed.translation.x >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;

//            float maxSpeedForTargetY = sqrt(2.f * abs(requestedSpeed.translation.y) * p.speedMaxChange.translation.y);
//            if(abs(requestedSpeed.translation.y) > maxSpeedForTargetY)
//              requestedSpeed.translation.y = requestedSpeed.translation.y >= 0.f ? maxSpeedForTargetY : -maxSpeedForTargetY;

//            float maxSpeedForTargetR = sqrt(2.f * abs(requestedSpeed.rotation) * p.speedMaxChange.rotation);
//            if(abs(requestedSpeed.rotation) > maxSpeedForTargetR)
//              requestedSpeed.rotation = requestedSpeed.rotation >= 0.f ? maxSpeedForTargetR : -maxSpeedForTargetR;
//          }

          // max speed change clipping (y-only)
          // just clip y and r since x will be clipped by min/maxRX in computeRefZMP
          requestedSpeed.translation.y = Range<>(lastSelectedSpeed.translation.y - p.speedMaxChange.translation.y, lastSelectedSpeed.translation.y + p.speedMaxChange.translation.y).limit(requestedSpeed.translation.y);
          requestedSpeed.rotation = Range<>(lastSelectedSpeed.rotation - p.speedMaxChange.rotation, lastSelectedSpeed.rotation + p.speedMaxChange.rotation).limit(requestedSpeed.rotation);

          // clip requested walk speed to the computed max speeds
          if(abs(requestedSpeed.rotation) > maxSpeed.rotation)
            requestedSpeed.rotation = requestedSpeed.rotation > 0.f ? maxSpeed.rotation : -maxSpeed.rotation;
          if(abs(requestedSpeed.translation.x) > maxSpeed.translation.x)
            requestedSpeed.translation.x = requestedSpeed.translation.x > 0.f ? maxSpeed.translation.x : -maxSpeed.translation.x;
          if(abs(requestedSpeed.translation.y) > maxSpeed.translation.y)
            requestedSpeed.translation.y = requestedSpeed.translation.y > 0.f ? maxSpeed.translation.y : -maxSpeed.translation.y;

//          // clip requested walk speed to a target walk speed limit
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//          {
//            requestedSpeed.translation.x = Range<>(-p.speedMax.translation.x * theMotionRequest.walkRequest.speed.translation.x, p.speedMax.translation.x * theMotionRequest.walkRequest.speed.translation.x).limit(requestedSpeed.translation.x);
//            requestedSpeed.translation.y = Range<>(-p.speedMax.translation.y * theMotionRequest.walkRequest.speed.translation.y, p.speedMax.translation.y * theMotionRequest.walkRequest.speed.translation.y).limit(requestedSpeed.translation.y);
//            requestedSpeed.rotation = Range<>(-p.speedMax.rotation * theMotionRequest.walkRequest.speed.rotation, p.speedMax.rotation * theMotionRequest.walkRequest.speed.rotation).limit(requestedSpeed.rotation);
//          }

          // generate step size from requested walk speed
          next.s = StepSize(requestedSpeed.rotation, requestedSpeed.translation.x * 0.5f, requestedSpeed.translation.y);

          // adjust step duration according to the actual desired step size
          {
            // do this before the "just move the outer foot" clipping
            const float accClippedSpeedX = Range<>(lastSelectedSpeed.translation.x - p.speedMaxChange.translation.x, lastSelectedSpeed.translation.x + p.speedMaxChange.translation.x).limit(requestedSpeed.translation.x);
            const float accClippedStepSizeX = accClippedSpeedX * 0.5f;

            {
              const float xxSpeedFactor = (p.teAtFullSpeedX - p.te) / (p.speedMax.translation.x * 0.5f);
              const float yySpeedFactor = (p.teAtFullSpeedY - p.te) / p.speedMax.translation.y;
              next.te += abs(next.s.translation.y) * yySpeedFactor;
              next.te += abs(accClippedStepSizeX) * xxSpeedFactor;
              next.tb = -next.te;
            }

            {
              float xSpeedFactor = (p.walkRefXAtFullSpeedX -  p.walkRefX) / (p.speedMax.translation.x * 0.5f);
              next.r.x += abs(accClippedStepSizeX) * xSpeedFactor;
              next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
              next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
            }

            {
              float walkRefYLimitMax = p.walkRefYLimit.max;
              float walkRefYLimitMin = p.walkRefYLimit.min;
              {
                float xSpeedFactor = (p.walkRefYLimitAtFullSpeedX.max -  p.walkRefYLimit.max) / (p.speedMax.translation.x * 0.5f);
                walkRefYLimitMax += abs(accClippedStepSizeX) * xSpeedFactor;
              }
              {
                float xSpeedFactor = (p.walkRefYLimitAtFullSpeedX.min -  p.walkRefYLimit.min) / (p.speedMax.translation.x * 0.5f);
                walkRefYLimitMin += abs(accClippedStepSizeX) * xSpeedFactor;
              }

              float ySpeedFactor = (p.walkRefYAtFullSpeedY -  p.walkRefY) / p.speedMax.translation.y;
              float xSpeedFactor = (p.walkRefYAtFullSpeedX -  p.walkRefY) / (p.speedMax.translation.x * 0.5f);
              next.r.y += (abs(requestedSpeed.translation.y) * ySpeedFactor + abs(accClippedStepSizeX) * xSpeedFactor) * (-sign);
              next.rYLimit.max = abs(next.r.y) + walkRefYLimitMax;
              next.rYLimit.min = abs(next.r.y) + walkRefYLimitMin;
            }
          }

          // just move the outer foot, when walking sidewards or when rotating
          if((next.s.translation.y < 0.f && nextSupportLeg == left) || (next.s.translation.y > 0.f && nextSupportLeg != left))
            next.s.translation.y = 0.f;
          if((next.s.rotation < 0.f && nextSupportLeg == left) || (next.s.rotation > 0.f && nextSupportLeg != left))
            next.s.rotation = 0.f;
//          if((next.s.translation.y < 0.f && nextSupportLeg != left) || (next.s.translation.y > 0.f && nextSupportLeg == left))
//            next.s.translation.y = 0.f;
//          if((next.s.rotation < 0.f && nextSupportLeg != left) || (next.s.rotation > 0.f && nextSupportLeg == left))
//            next.s.rotation = 0.f;

          // clip to walk target
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//          {
//            if((next.s.translation.x > 0.f && walkTarget.translation.x > 0.f && next.s.translation.x * p.odometryUpcomingScale.translation.x > walkTarget.translation.x) || (next.s.translation.x < 0.f && walkTarget.translation.x < 0.f && next.s.translation.x * p.odometryUpcomingScale.translation.x < walkTarget.translation.x))
//              next.s.translation.x = walkTarget.translation.x / p.odometryUpcomingScale.translation.x;
//            if((next.s.translation.y > 0.f && walkTarget.translation.y > 0.f && next.s.translation.y * p.odometryUpcomingScale.translation.y > walkTarget.translation.y) || (next.s.translation.y < 0.f && walkTarget.translation.y < 0.f && next.s.translation.y * p.odometryUpcomingScale.translation.y < walkTarget.translation.y))
//              next.s.translation.y = walkTarget.translation.y / p.odometryUpcomingScale.translation.y;
//            if((next.s.rotation > 0.f && walkTarget.rotation > 0.f && next.s.rotation * p.odometryUpcomingScale.rotation > walkTarget.rotation) || (next.s.rotation < 0.f && walkTarget.rotation < 0.f && next.s.rotation * p.odometryUpcomingScale.rotation < walkTarget.rotation))
//              next.s.rotation = walkTarget.rotation / p.odometryUpcomingScale.rotation;
//          }
        }

        next.lRotation = Vector3<>(
                           p.walkLiftRotation.x * sign * fabs(next.s.translation.y) / p.speedMax.translation.y,
                           next.s.translation.x > 0.f ? (p.walkLiftRotation.y * next.s.translation.x / (p.speedMax.translation.x * 0.5f)) : 0,
                           p.walkLiftRotation.z * sign);

        {
          float xSpeedFactor = (p.walkLiftOffsetAtFullSpeedY.x - p.walkLiftOffset.x) / p.speedMax.translation.y;
          float ySpeedFactor = (p.walkLiftOffsetAtFullSpeedY.y - p.walkLiftOffset.y) / p.speedMax.translation.y;
          float zSpeedFactor = (p.walkLiftOffsetAtFullSpeedY.z - p.walkLiftOffset.z) / p.speedMax.translation.y;
          next.l.x += abs(next.s.translation.y) * xSpeedFactor;
          next.l.y += abs(next.s.translation.y) * ySpeedFactor * sign;
          next.l.z += abs(next.s.translation.y) * zSpeedFactor;
        }

        {
          float xSpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.x - p.walkAntiLiftOffset.x) / p.speedMax.translation.y;
          float ySpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.y - p.walkAntiLiftOffset.y) / p.speedMax.translation.y;
          float zSpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.z - p.walkAntiLiftOffset.z) / p.speedMax.translation.y;
          next.al.x += abs(next.s.translation.y) * xSpeedFactor;
          next.al.y += abs(next.s.translation.y) * ySpeedFactor * sign;
          next.al.z += abs(next.s.translation.y) * zSpeedFactor;
        }
      }

      lastSelectedSpeed = Pose2D(lastStepSize.rotation + next.s.rotation, lastStepSize.translation.x + next.s.translation.x, lastStepSize.translation.y + next.s.translation.y);

      // next.r.y + next.x0.y * cosh(next.k * next.tb) = 0.f
      // => next.x0.y = - next.r.y / cosh(next.k * next.tb)
      next.x0 = Vector2<>(0.f, -next.r.y / cosh(next.k.y * next.tb));

      // next.xv0.x * sinh(next.k * next.tb) / next.k = next.s.translation.x * -0.5f
      // => next.xv0.x = next.s.translation.x * -0.5f * next.k / sinh(next.k * next.tb)
      next.xv0 = Vector2<>(next.s.translation.x * -0.5f * next.k.x / sinh(next.k.x * next.tb), 0.f);

      // next.r.y + next.x0.y * cosh(next.k * next.tb) = next.s.translation.y * -0.5f
      // => next.tb = -acosh((next.s.translation.y * -0.5f - next.r.y) / next.x0.y) / next.k
      next.tb = -saveAcosh((next.s.translation.y * -0.5f - next.r.y) / next.x0.y) / next.k.y;

      // next.r.x + next.xv0.x * sinh(next.k * next.tb) / k  = next.xtb.x
      next.xtb = Vector2<>(next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x, next.s.translation.y * -0.5f);

      // next.xvtb.x = next.xv0.x * cosh(next.k * next.tb)
      // next.xvtb.y = next.x0.y * next.k * sinh(next.k * next.tb)
      next.xvtb = Vector2<>(next.xv0.x * cosh(next.k.x * next.tb), next.x0.y * next.k.y * sinh(next.k.y * next.tb));

      next.originalRX = next.r.x;
    }
    nextPendulumParameters = next;
  }
}

void WalkingEngine::computeOdometryOffset()
{
//  if(p.odometryUseTorsoMatrix)
//  {
//    // "measured" odometry
//    if(lastTorsoMatrix.translation.z != 0.)
//    {
//      Pose3D odometryOffset3D(lastTorsoMatrix);
//      odometryOffset3D.conc(theTorsoMatrix.offset);
//      odometryOffset3D.conc(theTorsoMatrix.invert());
//      odometryOffset.translation.x = odometryOffset3D.translation.x;
//      odometryOffset.translation.y = odometryOffset3D.translation.y;
//      odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
//    }
//    lastTorsoMatrix = theTorsoMatrix;
//  }
//  else
//  {
//    // calculated odometry
//    if(observedPendulumPlayer.supportLeg == lastSupportLeg)
//      odometryOffset = (stepOffset - lastStepOffset) * 0.5f;
//    else
//      odometryOffset = (stepOffset + observedPendulumPlayer.s * 2.f - lastStepOffset) * 0.5f; // == ((observedPendulumPlayer.s - lastStepOffset) + (stepOffset - (observedPendulumPlayer.s * -1f))) * 0.5f;
//  }

//  upcomingOdometryOffset = observedPendulumPlayer.next.s - stepOffset * 0.5f; // == (observedPendulumPlayer.next.s - stepOffset) * 0.5f + observedPendulumPlayer.next.s * 0.5f

//  // HACK: somehow this improves the accuracy of the upcoming odometry offset for target walks (but i have no idea why)
//  upcomingOdometryOffset -= (observedPendulumPlayer.s + observedPendulumPlayer.next.s) * 0.5f;

//  float sign = observedPendulumPlayer.supportLeg == left ? -1.f : 1.f;
//  Pose2D up(p.odometryUpcomingOffset.rotation * sign, p.odometryUpcomingOffset.translation.x, p.odometryUpcomingOffset.translation.y * sign);
//  upcomingOdometryOffset += up;
//  upcomingOdometryOffsetValid = observedPendulumPlayer.supportLeg == pendulumPlayer.supportLeg;
//  if(!upcomingOdometryOffsetValid)
//    upcomingOdometryOffset += pendulumPlayer.next.s;
//  else
//    upcomingOdometryOffsetValid = (observedPendulumPlayer.te - observedPendulumPlayer.t) > 0.040f;

//  lastSupportLeg = observedPendulumPlayer.supportLeg;
//  lastStepOffset = stepOffset;

//  odometryOffset.translation.x *= p.odometryScale.translation.x;
//  odometryOffset.translation.y *= p.odometryScale.translation.y;
//  odometryOffset.rotation *= p.odometryScale.rotation;

//  upcomingOdometryOffset.translation.x *= p.odometryUpcomingScale.translation.x;
//  upcomingOdometryOffset.translation.y *= p.odometryUpcomingScale.translation.y;
//  upcomingOdometryOffset.rotation *= p.odometryUpcomingScale.rotation;

//  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//    requestedWalkTarget -= odometryOffset;
}

void WalkingEngine::ObservedPendulumPlayer::init(StepType stepType, float t, SupportLeg supportLeg, const Vector2<>& r, const Vector2<>& x0, const Vector2<>& k, float deltaTime)
{
  Parameters& p = walkingEngine->p;

  active = true;
  launching = true;

  this->supportLeg = supportLeg;

  this->l = this->al = Vector3<>();
  this->s = StepSize();
  this->type = stepType;

  this->k = k;
  this->te = p.te;
  this->tb = -p.te;
  this->t = this->tb + t;
  this->xv0 = Vector2<>();
  this->x0 = x0;
  this->r = r;
  this->originalRX  = r.x;
  this->c = Vector2<>();
  this->sXLimit.max = p.speedMax.translation.x * (1.1f * 0.5f);
  this->sXLimit.min = p.speedMaxBackwards * (-1.1f * 0.5f);
  this->rXLimit.max = this->r.x + p.walkRefXSoftLimit.max;
  this->rXLimit.min = this->r.x + p.walkRefXSoftLimit.min;
  this->rYLimit.max = p.walkRefY + p.walkRefYLimit.max;
  this->rYLimit.min = p.walkRefY + p.walkRefYLimit.min;

  cov = b_human::Matrix4x4f(
          Vector4f(p.observerProcessDeviation[0]*p.observerProcessDeviation[0], 0.f, p.observerProcessDeviation[0] * p.observerProcessDeviation[2], 0.f),
          Vector4f(0.f, p.observerProcessDeviation[1]*p.observerProcessDeviation[1], 0.f, p.observerProcessDeviation[1] * p.observerProcessDeviation[3]),
          Vector4f(p.observerProcessDeviation[0] * p.observerProcessDeviation[2], 0.f, p.observerProcessDeviation[2]*p.observerProcessDeviation[2], 0.f),
          Vector4f(0.f, p.observerProcessDeviation[1] * p.observerProcessDeviation[3], 0.f, p.observerProcessDeviation[3]*p.observerProcessDeviation[3]));

  generateNextStepSize();

  computeSwapTimes(this->tb, 0.f, 0.f, 0.f);

  computeRefZmp(this->tb, r.x, 0.f, 0.f);
}

void WalkingEngine::PendulumPlayer::seek(float deltaT)
{
  t += deltaT;
  if(t >= 0.f)
  {
    launching = false;
    if(t >= te)
    {
      if(type == toStand || type == toStandLeft || type == toStandRight)
      {
        active = false;
        return;
      }

      float const xTeY = r.y + c.y * te + x0.y * cosh(k.y * te) + xv0.y * sinh(k.y * te) / k.y;
      float const xvTeY = c.y + k.y * x0.y * sinh(k.y * te) + xv0.y * cosh(k.y * te);
      float const xTeX = r.x + c.x * te + x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
      float const xvTeX = c.x + k.x * x0.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);

      supportLeg = supportLeg == left ? right : left;
      t = next.tb + (t - te);
      (PendulumParameters&)*this = next;
      generateNextStepSize();

      computeSwapTimes(tb, xTeY - s.translation.y, xvTeY, 0.f);
      computeRefZmp(tb, xTeX - s.translation.x, xvTeX, 0.f);
    }
  }
}

void WalkingEngine::ObservedPendulumPlayer::applyCorrection(const Vector3<>& leftError, const Vector3<>& rightError, float deltaTime)
{
  Parameters& p = walkingEngine->p;
  Vector3<> error;
  switch(p.observerErrorMode)
  {
  case Parameters::indirect:
    error = supportLeg != left ? leftError : rightError;
    break;
  case Parameters::direct:
    error = supportLeg == left ? leftError : rightError;
    break;
  case Parameters::mixed:
  {
    float x = (t - tb) / (te - tb);
    if(x < 0.5f)
      x = 0.f;
    else
      x = (x - 0.5f) * 2.f;
    if(supportLeg != left)
      x = 1.f - x;
    error = leftError * (1.f - x) + rightError * x;
  }
  default:
    error = (leftError + rightError) * 0.5f;
    break;
  }

  static const b_human::Matrix2x4f c(Vector2f(1, 0), Vector2f(0, 1), Vector2f(), Vector2f());
  static const b_human::Matrix4x2f cTransposed = c.transpose();
  static const b_human::Matrix4x4f a(Vector4f(1, 0, 0, 0), Vector4f(0, 1, 0, 0),
                            Vector4f(deltaTime, 0, 1, 0), Vector4f(0, deltaTime, 0, 1));
  static const b_human::Matrix4x4f aTransponsed = a.transpose();

  cov = a * cov * aTransponsed;

  for(int i = 0; i < 4; ++i)
    cov[i][i] += p.observerProcessDeviation[i]*p.observerProcessDeviation[i];

  b_human::Matrix2x2f covPlusSensorCov = c * cov * cTransposed;
  Vector2f observerMeasurementDeviation = p.observerMeasurementDeviation;
  if(walkingEngine->instable)
    observerMeasurementDeviation = p.observerMeasurementDeviationWhenInstable;
  else if(next.s.translation.x > 0.f)
  {
    observerMeasurementDeviation.x += (p.observerMeasurementDeviationAtFullSpeedX.x - p.observerMeasurementDeviation.x) * abs(next.s.translation.x) / (p.speedMax.translation.x * 0.5f);
    observerMeasurementDeviation.y += (p.observerMeasurementDeviationAtFullSpeedX.y - p.observerMeasurementDeviation.y) * abs(next.s.translation.x) / (p.speedMax.translation.x * 0.5f);
  }
  covPlusSensorCov[0][0] += observerMeasurementDeviation[0]*observerMeasurementDeviation[0];
  covPlusSensorCov[1][1] += observerMeasurementDeviation[1]*observerMeasurementDeviation[1];
  b_human::Matrix4x2f kalmanGain = cov * cTransposed * covPlusSensorCov.invert();
  Vector2f innovation(error.x, error.y);
  Vector4f correction = kalmanGain * innovation;
  cov -= kalmanGain * c * cov;

  // compute updated xt and xvt
  Vector2<> xt(
    r.x + this->c.x * t + x0.x * cosh(k.x * t) + xv0.x * sinh(k.x * t) / k.x + correction[0],
    r.y + this->c.y * t + x0.y * cosh(k.y * t) + xv0.y * sinh(k.y * t) / k.y + correction[1]);
  Vector2<> xvt(
    this->c.x + k.x * x0.x * sinh(k.x * t) + xv0.x * cosh(k.x * t) + correction[2],
    this->c.y + k.y * x0.y * sinh(k.y * t) + xv0.y * cosh(k.y * t) + correction[3]);

  computeSwapTimes(t, xt.y, xvt.y, error.y);
  computeRefZmp(t, xt.x, xvt.x, error.x);
}

void WalkingEngine::PendulumPlayer::generateNextStepSize()
{
  walkingEngine->generateNextStepSize(supportLeg == right ? left : right, type, next);
}



void WalkingEngine::PendulumPlayer::computeSwapTimes(float t, float xt, float xvt, float errory)
{
  if(te - t < 0.005f)
    return;

  switch(type)
  {
  case toStand:
  case toStandLeft:
  case toStandRight:
  case fromStand:
  case fromStandLeft:
  case fromStandRight:
  {
    float const xte = next.s.translation.y + next.xtb.y;
    float const xvte = next.xvtb.y;

    // r * 1 + c * te + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  = xte
    //       + c * 1  + x0 * k * sinh(k * te) + xv0 * cosh(k * te)      = xvte
    // r * 1 + c * t  + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k   = xt
    //       + c * 1  + x0 * k * sinh(k * t)  + xv0 * cosh(k * t)       = xvt

    b_human::Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 1.f, 0.f),
      Vector<4>(te, 1.f, t, 1.f),
      Vector<4>(cosh(k.y * te), k.y * sinh(k.y * te), cosh(k.y * t), k.y * sinh(k.y * t)),
      Vector<4>(sinh(k.y * te) / k.y, cosh(k.y * te), sinh(k.y * t) / k.y, cosh(k.y * t)));
    Vector<4> b(xte, xvte, xt, xvt);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    r.y = x[0];
    c.y = x[1];
    x0.y = x[2];
    xv0.y = x[3];
  }
  return;
  default:
    break;
  }

  Parameters& p = walkingEngine->p;

  if(errory != 0.f && walkingEngine->balanceStepSize.y != 0.f && !walkingEngine->m_pedantic/*&& kickType == WalkRequest::none /*&& !walkingEngine->theMotionRequest.walkRequest.pedantic*/)
  {
    assert(next.xv0.y == 0.f);
    float sy = next.xtb.y * -2.f;
    sy += errory * (walkingEngine->instable ? p.balanceStepSizeWhenInstable.y : walkingEngine->balanceStepSize.y);
    next.tb = -saveAcosh((sy * -0.5f - next.r.y) / next.x0.y) / next.k.y;
    next.xtb.y =  sy * -0.5f;
    next.xvtb.y =  next.x0.y * next.k.y * sinh(next.k.y * next.tb);
  }

  assert(next.xv0.y == 0.f);
  //float const xte = next.s.translation.y + next.xtb.y;
  float const xvte = next.xvtb.y;

  //           x0.y * k * sinh(k * te) + xv0.y * cosh(k * te)     = xvte
  // r.y * 1 + x0.y * cosh(k * t)      + xv0.y * sinh(k * t) / k  = xt
  //           x0.y * k * sinh(k * t)  + xv0.y * cosh(k * t)      = xvt

  b_human::Matrix<3, 3> a(
    Vector<3>(0.f, 1.f, 0.f),
    Vector<3>(k.y * sinh(k.y * te), cosh(k.y * t), k.y * sinh(k.y * t)),
    Vector<3>(cosh(k.y * te), sinh(k.y * t) / k.y, cosh(k.y * t)));
  Vector<3> b(xvte, xt, xvt);

  Vector<3> x;
  if(!a.solve(b, x))
  {
    assert(false);
    return;
  }

  r.y = x[0];
  c.y = 0.f;
  x0.y = x[1];
  xv0.y = x[2];

  float newXte = r.y  + x0.y * cosh(k.y * te) + xv0.y * sinh(k.y * te) / k.y;
  next.s.translation.y = newXte - next.xtb.y;
}

void WalkingEngine::PendulumPlayer::computeRefZmp(float t, float xt, float xvt, float errorx)
{
  if(te - t < 0.005f)
    return;

  switch(type)
  {
  case toStand:
  case toStandLeft:
  case toStandRight:
  {
    float const xte = next.s.translation.x + next.xtb.x;
    float const xvte = next.xvtb.x;

    // r * 1 + c * te + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  = xte
    //       + c * 1  + x0 * k * sinh(k * te) + xv0 * cosh(k * te)      = xvte
    // r * 1 + c * t  + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k   = xt
    //       + c * 1  + x0 * k * sinh(k * t)  + xv0 * cosh(k * t)       = xvt

    b_human::Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 1.f, 0.f),
      Vector<4>(te, 1.f, t, 1.f),
      Vector<4>(cosh(k.x * te), k.x * sinh(k.x * te), cosh(k.x * t), k.x * sinh(k.x * t)),
      Vector<4>(sinh(k.x * te) / k.x, cosh(k.x * te), sinh(k.x * t) / k.x, cosh(k.x * t)));
    Vector<4> b(xte, xvte, xt, xvt);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    r.x = x[0];
    c.x = x[1];
    x0.x = x[2];
    xv0.x = x[3];
  }
  return;
  default:
    break;
  }

  Parameters& p = walkingEngine->p;
  if(errorx != 0.f && walkingEngine->balanceStepSize.x != 0.f && !walkingEngine->m_pedantic/*&& kickType == WalkRequest::none  /*&& !walkingEngine->theMotionRequest.walkRequest.pedantic */)
  {
    assert(next.x0.x == 0.f);
    float sx = next.xv0.x * sinh(next.k.x * next.tb) / (-0.5f * next.k.x);
    sx += errorx * (walkingEngine->instable ? p.balanceStepSizeWhenInstable.x : walkingEngine->balanceStepSize.x);
    next.xv0.x = sx * -0.5f * next.k.x / sinh(next.k.x * next.tb);
    next.xtb.x = next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x;
    next.xvtb.x = next.xv0.x * cosh(next.k.x * next.tb);
  }


  assert(next.x0.x == 0.f);
  //float const xte = next.s.translation.x + next.xtb.x;
  float const xvte = next.xvtb.x;

  //           x0.x * k * sinh(k * te) + xv0.x * cosh(k * te)     = xvte
  // r.x * 1 + x0.x * cosh(k * t)      + xv0.x * sinh(k * t) / k  = xt
  //           x0.x * k * sinh(k * t)  + xv0.x * cosh(k * t)      = xvt

  b_human::Matrix<3, 3> a(
    Vector<3>(0.f, 1.f, 0.f),
    Vector<3>(k.x * sinh(k.x * te), cosh(k.x * t), k.x * sinh(k.x * t)),
    Vector<3>(cosh(k.x * te), sinh(k.x * t) / k.x, cosh(k.x * t)));
  Vector<3> b(xvte, xt, xvt);

  Vector<3> x;
  if(!a.solve(b, x))
  {
    assert(false);
    return;
  }

  r.x = x[0];
  c.x = 0.f;
  x0.x = x[1];
  xv0.x = x[2];

  float newXte = r.x +  x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
  float newXvte = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
  float newNextXvtb = newXvte;
  float newNextXv0 = newNextXvtb / cosh(next.k.x * next.tb);
  float newNextXtb = next.r.x + newNextXv0 * sinh(next.k.x * next.tb) / next.k.x;
  next.s.translation.x = newXte - newNextXtb;

  if(!rXLimit.isInside(r.x))
  {
    r.x = rXLimit.limit(r.x);

    // x0.x * cosh(k * t)      + xv0.x * sinh(k * t) / k  = xt - r.x
    // x0.x * k * sinh(k * t)  + xv0.x * cosh(k * t)      = xvt

    b_human::Matrix<2, 2> a(
      Vector<2>(cosh(k.x * t), k.x * sinh(k.x * t)),
      Vector<2>(sinh(k.x * t) / k.x, cosh(k.x * t)));
    Vector<2> b(xt - r.x, xvt);

    Vector<2> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    x0.x = x[0];
    xv0.x = x[1];

    float newXte = r.x +  x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
    float newXvte = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
    float newNextXvtb = newXvte;
    float newNextXv0 = newNextXvtb / cosh(next.k.x * next.tb);
    float newNextXtb = next.r.x + newNextXv0 * sinh(next.k.x * next.tb) / next.k.x;
    //if(kickType == WalkRequest::none)
    next.s.translation.x = newXte - newNextXtb;
    if(type == unknown)
    {
      next.xv0.x = newNextXv0;
      next.xtb.x = newNextXtb;
      next.xvtb.x = newNextXvtb;
    }
  }

  if(!sXLimit.isInside(next.s.translation.x)/* && kickType == WalkRequest::none*/)
  {
    next.s.translation.x = sXLimit.limit(next.s.translation.x);

    // r + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k                                                                     = xt
    //     x0 * k * sinh(k * t)  + xv0 * cosh(k * t)                                                                         = xvt
    //     x0 * k * sinh(k * te) + xv0 * cosh(k * te)          - nx0 * nk * sinh(nk * ntb) - nxv0 * cosh(nk * ntb)           = 0      // <=> xvte = nxvtb
    // r + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k - nr - nx0 * cosh(nk * ntb)      - nxv0 * sinh(nk * ntb) / nk - ns = 0      // <=> xte - nxtb = ns

    // nx0 = 0

    // r + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k                                = xt
    //     x0 * k * sinh(k * t)  + xv0 * cosh(k * t)                                    = xvt
    //     x0 * k * sinh(k * te) + xv0 * cosh(k * te)      - nxv0 * cosh(nk * ntb)      = 0
    // r + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  - nxv0 * sinh(nk * ntb) / nk = ns + nr

    b_human::Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 0.f, 1.f),
      Vector<4>(cosh(k.x * t), k.x * sinh(k.x * t), k.x * sinh(k.x * te), cosh(k.x * te)),
      Vector<4>(sinh(k.x * t) / k.x, cosh(k.x * t), cosh(k.x * te), sinh(k.x * te) / k.x),
      Vector<4>(0.f, 0.f, -cosh(next.k.x * next.tb), -sinh(next.k.x * next.tb) / next.k.x));
    Vector<4> b(xt, xvt, 0, next.s.translation.x + next.r.x);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    r.x = x[0];
    x0.x = x[1];
    xv0.x = x[2];

    if(type == unknown)
    {
      next.xvtb.x = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
      next.xv0.x = next.xvtb.x / cosh(next.k.x * next.tb);
      next.xtb.x = next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x;
    }
  }

  if(type == unknown)
  {
    Parameters& p = walkingEngine->p;
    rXLimit.max = originalRX + p.walkRefXLimit.max;
    if(rXLimit.max > p.walkRefXAtFullSpeedX + p.walkRefXLimit.max)
      rXLimit.max = p.walkRefXAtFullSpeedX + p.walkRefXLimit.max;
    rXLimit.min = originalRX + p.walkRefXLimit.min;
    type = normal;
  }
}

void WalkingEngine::PendulumPlayer::getStance(LegStance& stance, float* leftArmAngle, float* rightArmAngle, StepSize* stepOffset) const
{
  const float phase = (t - tb) / (te - tb);

  Parameters& p = walkingEngine->p;
  const float swingMoveFadeIn = phase < p.walkMovePhase.start ? 0.f :
                                phase > p.walkMovePhase.start + p.walkMovePhase.duration ? 1.f :
                                smoothShape((phase - p.walkMovePhase.start) / p.walkMovePhase.duration);
  const float swingMoveFadeOut = 1.f - swingMoveFadeIn;
  const float sl = phase < p.walkLiftPhase.start || phase > p.walkLiftPhase.start + p.walkLiftPhase.duration ? 0.f :
                   smoothShape((phase - p.walkLiftPhase.start) / p.walkLiftPhase.duration * 2.f);


  Vector3<> r(this->r.x + this->c.x * t, this->r.y + this->c.y * t, 0.f);
  Vector3<> refToCom(
    p.standComPosition.x + x0.x * cosh(k.x * t) + xv0.x * sinh(k.x * t) / k.x,
    x0.y * cosh(k.y * t) + xv0.y * sinh(k.y * t) / k.y,
    p.standComPosition.z);

  switch(type)
  {
  case toStandLeft:
  case toStandRight:
  case fromStandLeft:
  case fromStandRight:
  {
    const float ratio = smoothShape(type == toStandLeft || type == toStandRight ? 1.f - t / tb : 1.f - t / te);
    refToCom.z += ratio * (p.kickComPosition.z - p.standComPosition.z);
    refToCom.x += ratio * (p.kickComPosition.x - p.standComPosition.x);
//    std::cout << "case fromStandRight: refToCom = " << refToCom << std::endl;

  }
  break;
  default:
    refToCom += next.al * sl;
//    std::cout << "case default: refToCom = " << refToCom << std::endl;

    break;
  }

  if(supportLeg == left)
  {
    const Vector3<> rightStepOffsetTranslation = next.l * sl - (next.s.translation + s.translation) * swingMoveFadeOut;

    Vector3<> rightStepOffsetRotation = next.lRotation * sl;
    rightStepOffsetRotation.z += next.s.rotation * swingMoveFadeIn;
    const Vector3<> leftStepOffsetRotation(0.f, 0.f, s.rotation * swingMoveFadeOut);

    stance.leftOriginToCom = refToCom + r;
    stance.leftOriginToFoot = Pose3D(RotationMatrix(leftStepOffsetRotation), Vector3<>(0.f, p.standComPosition.y, 0.f));

    stance.rightOriginToCom = refToCom + r - next.s.translation;
    stance.rightOriginToFoot = Pose3D(RotationMatrix(rightStepOffsetRotation), Vector3<>(0.f, -p.standComPosition.y, 0.f) + rightStepOffsetTranslation);

    if(leftArmAngle)
      *leftArmAngle = (next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut) / p.speedMax.translation.x * p.walkArmRotation;
    if(rightArmAngle)
      *rightArmAngle = (s.translation.x * swingMoveFadeOut - next.s.translation.x * swingMoveFadeIn) / p.speedMax.translation.x * p.walkArmRotation;
  }
  else
  {
    const Vector3<> leftStepOffsetTranslation = next.l * sl - (next.s.translation + s.translation) * swingMoveFadeOut;

    Vector3<> leftStepOffsetRotation = next.lRotation * sl;
    leftStepOffsetRotation.z += next.s.rotation * swingMoveFadeIn;
    const Vector3<> rightStepOffsetRotation(0.f, 0.f, s.rotation * swingMoveFadeOut);

    stance.rightOriginToCom = refToCom + r;
    stance.rightOriginToFoot = Pose3D(RotationMatrix(rightStepOffsetRotation), Vector3<>(0.f, -p.standComPosition.y, 0.f));

    stance.leftOriginToCom = refToCom + r - next.s.translation;
    stance.leftOriginToFoot = Pose3D(RotationMatrix(leftStepOffsetRotation), Vector3<>(0.f, p.standComPosition.y, 0.f) + leftStepOffsetTranslation);

    if(rightArmAngle)
      *rightArmAngle = (next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut) / p.speedMax.translation.x * p.walkArmRotation;
    if(leftArmAngle)
      *leftArmAngle = (s.translation.x * swingMoveFadeOut - next.s.translation.x * swingMoveFadeIn) / p.speedMax.translation.x * p.walkArmRotation;
  }

  if(stepOffset)
  {
    stepOffset->translation.x = next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut;
    stepOffset->translation.y = next.s.translation.y * swingMoveFadeIn - s.translation.y * swingMoveFadeOut;
    stepOffset->translation.z = 0.f;
    stepOffset->rotation = next.s.rotation * swingMoveFadeIn - s.rotation * swingMoveFadeOut;
  }
}

float WalkingEngine::PendulumPlayer::smoothShape(float r) const
{
  switch(walkingEngine->p.walkFadeInShape)
  {
  case WalkingEngine::Parameters::sine:
    return 0.5f - cos(r * pi) * 0.5f;
  case WalkingEngine::Parameters::sqr:
    if(r > 1.f)
      r = 2.f - r;
    return r < 0.5f ? 2.f * r * r : (4.f - 2.f * r) * r - 1.f;
  default:
    assert(false);
    return 0;
  }
}

//WalkingEngine::KickPlayer::KickPlayer() : kick(0)
//{
//  assert((WalkRequest::numOfKickTypes - 1) % 2 == 0);
//  for(int i = 0; i < (WalkRequest::numOfKickTypes - 1) / 2; ++i)
//  {
//    char filePath[256];
//    sprintf(filePath, "Kicks/%s.cfg", WalkRequest::getName(WalkRequest::KickType(i * 2 + 1)));
//    kicks[i].load(filePath);
//  }
//}
//
//bool WalkingEngine::KickPlayer::isKickStandKick(WalkRequest::KickType type) const
//{
//  bool mirrored = (type - 1) % 2 != 0;
//  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
//  return kick.isStandKick();
//}
//
//void WalkingEngine::KickPlayer::getKickStepSize(WalkRequest::KickType type, float& rotation, Vector3<>& translation) const
//{
//  bool mirrored = (type - 1) % 2 != 0;
//  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
//  kick.getStepSize(rotation, translation);
//  if(mirrored)
//  {
//    translation.y = -translation.y;
//    rotation = -rotation;
//  }
//}
//
//void WalkingEngine::KickPlayer::getKickPreStepSize(WalkRequest::KickType type, float& rotation, Vector3<>& translation) const
//{
//  bool mirrored = (type - 1) % 2 != 0;
//  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
//  kick.getPreStepSize(rotation, translation);
//  if(mirrored)
//  {
//    translation.y = -translation.y;
//    rotation = -rotation;
//  }
//}
//
//float WalkingEngine::KickPlayer::getKickDuration(WalkRequest::KickType type) const
//{
//  bool mirrored = (type - 1) % 2 != 0;
//  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
//  return kick.getDuration();
//}
//
//float WalkingEngine::KickPlayer::getKickRefX(WalkRequest::KickType type, float defaultValue) const
//{
//  bool mirrored = (type - 1) % 2 != 0;
//  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
//  return kick.getRefX(defaultValue);
//}
//
//void WalkingEngine::KickPlayer::init(WalkRequest::KickType type, const Vector2<>& ballPosition, const Vector2<>& target)
//{
//  assert(!kick);
//  mirrored = (type - 1) % 2 != 0;
//  this->type = type;
//  kick = &kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
//  setParameters(ballPosition, target);
//  kick->init();
//}
//
//void WalkingEngine::KickPlayer::seek(float deltaT)
//{
//  if(kick)
//    if(!kick->seek(deltaT))
//      kick = 0;
//}
//
//float WalkingEngine::KickPlayer::getLength() const
//{
//  if(kick)
//    return kick->getLength();
//  assert(false);
//  return -1.f;
//}
//
//float WalkingEngine::KickPlayer::getCurrentPosition() const
//{
//  if(kick)
//    return kick->getCurrentPosition();
//  assert(false);
//  return -1.f;
//}
//
//void WalkingEngine::KickPlayer::apply(Stance& stance)
//{
//  if(!kick)
//    return;
//  Vector3<> additionalFootRotation;
//  Vector3<> additionFootTranslation;
//  float additionHeadAngles[2];
//  float additionLeftArmAngles[4];
//  float additionRightArmAngles[4];
//
//  for(int i = 0; i < 2; ++i)
//    additionHeadAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::headYaw + i), 0.f);
//  for(int i = 0; i < 4; ++i)
//  {
//    additionLeftArmAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::lShoulderPitch + i), 0.f);
//    additionRightArmAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::rShoulderPitch + i), 0.f);
//  }
//  for(int i = 0; i < 3; ++i)
//  {
//    additionFootTranslation[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::footTranslationX + i), 0.f);
//    additionalFootRotation[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::footRotationX + i), 0.f);
//  }
//
//  if(mirrored)
//  {
//    additionalFootRotation.x = -additionalFootRotation.x;
//    additionalFootRotation.z = -additionalFootRotation.z;
//    additionFootTranslation.y = -additionFootTranslation.y;
//
//    for(unsigned int i = 0; i < sizeof(stance.leftArmJointAngles) / sizeof(*stance.leftArmJointAngles); ++i)
//    {
//      float tmp = additionLeftArmAngles[i];
//      additionLeftArmAngles[i] = additionRightArmAngles[i];
//      additionRightArmAngles[i] = tmp;
//    }
//    additionHeadAngles[0] = -additionHeadAngles[0];
//  }
//
//  (mirrored ? stance.rightOriginToFoot : stance.leftOriginToFoot).conc(Pose3D(RotationMatrix(additionalFootRotation), additionFootTranslation));
//  for(int i = 0; i < 2; ++i)
//    if(stance.headJointAngles[i] != JointData::off)
//      stance.headJointAngles[i] += additionHeadAngles[i];
//  for(int i = 0; i < 4; ++i)
//  {
//    stance.leftArmJointAngles[i] += additionLeftArmAngles[i];
//    stance.rightArmJointAngles[i] += additionRightArmAngles[i];
//  }
//}
//
//void WalkingEngine::KickPlayer::setParameters(const Vector2<>& ballPosition, const Vector2<>& target)
//{
//  if(!kick)
//    return;
//  if(mirrored)
//    kick->setParameters(Vector2<>(ballPosition.x, -ballPosition.y), Vector2<>(target.x, -target.y));
//  else
//    kick->setParameters(ballPosition, target);
//}
//
//bool WalkingEngine::KickPlayer::handleMessage(InMessage& message)
//{
//  if(message.getMessageID() == idWalkingEngineKick)
//  {
//    unsigned int id, size;
//    message.bin >> id >> size;
//    assert(id < WalkRequest::numOfKickTypes);
//    char* buffer = new char[size + 1];
//    message.bin.read(buffer, size);
//    buffer[size] = '\0';
//    char filePath[256];
//    sprintf(filePath, "Kicks/%s.cfg", WalkRequest::getName(WalkRequest::KickType(id)));
//    if(kicks[(id - 1) / 2].load(filePath, buffer))
//    {
//      OUTPUT(idText, text, filePath << ": ok");
//    }
//    delete[] buffer;
//    return true;
//  }
//  else
//    return false;
//}

Matrix WalkingEngine::Pose2Matrix(const Pose3D& pose)
{
    Matrix result(4,4,true);
    result[0][0] = pose.rotation.c0.x;
    result[1][0] = pose.rotation.c0.y;
    result[2][0] = pose.rotation.c0.z;

    result[0][1] = pose.rotation.c1.x;
    result[1][1] = pose.rotation.c1.y;
    result[2][1] = pose.rotation.c1.z;

    result[0][2] = pose.rotation.c2.x;
    result[1][2] = pose.rotation.c2.y;
    result[2][2] = pose.rotation.c2.z;

    result[0][3] = pose.translation.x;
    result[1][3] = pose.translation.y;
    result[2][3] = pose.translation.z;

    return result;
}
