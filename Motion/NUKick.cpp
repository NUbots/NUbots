/*! @file NUKick.cpp
    @brief Implementation of nukick class

    @author Jed Rietveld
 
 Copyright (c) 2010 Jed Rietveld
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NUKick.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Behaviour/Jobs/MotionJobs/KickJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "NUPlatform/NUSystem.h"
#include "Motion/Tools/MotionCurves.h"

#include "motionconfig.h"
#include "debugverbositynumotion.h"

#ifdef USE_WALK
#include "Motion/NUWalk.h"
#endif // USE_WALK

#include "debug.h"
#include "debugverbositynumotion.h"
#include "Tools/Math/General.h"
using namespace mathGeneral;

//#if DEBUG_LOCALISATION_VERBOSITY > 0



NUKick::NUKick(NUWalk* walk)
{
    m_walk = walk;
    m_kinematicModel = new Kinematics();
    m_kinematicModel->LoadModel(std::string("None"));
    pose = DO_NOTHING;
    m_kickingLeg = noLeg;

    m_kickIsActive = false;
    m_stateCommandGiven = false;
    m_estimatedStateCompleteTime = 0.0;
    m_currentTimestamp = 0;
    m_previousTimestamp = 0;
    loadKickParameters();

    m_pauseState = false;
    m_variableGainValue = 0.01;
    m_armCommandSent = false;
}

/*! @brief Destructor for motion module
 */
NUKick::~NUKick()
{
    kill();
    delete m_kinematicModel;
}

void NUKick::loadKickParameters()
{
    m_defaultMotorGain = 75.0f;                 // Default to 75% gain.
    m_leftLegInitialPose.resize(6,0.0f);
    m_leftLegInitialPose[3] = 0.5;
    m_leftLegInitialPose[1] = - m_leftLegInitialPose[3] / 2.0f;
    m_leftLegInitialPose[5] = - m_leftLegInitialPose[3] / 2.0f;
    m_rightLegInitialPose.assign(m_leftLegInitialPose.begin(), m_leftLegInitialPose.end());

    jointLimit hyp = jointLimit(-1.145303, 0.740810);

    m_leftLegLimits.reserve(6);
    // Hip Roll
    m_leftLegLimits.push_back(jointLimit(-0.379472, 0.790477));
    // Hip Pitch
    m_leftLegLimits.push_back(jointLimit(-1.773912, 0.484090));
    // Hip Yaw - Pitch
    m_leftLegLimits.push_back(hyp);
    // Knee Pitch
    m_leftLegLimits.push_back(jointLimit(-0.092346, 2.112528));
    // Ankle Roll
    m_leftLegLimits.push_back(jointLimit(-0.769001, 0.397880));
    // Ankle Pitch
    m_leftLegLimits.push_back(jointLimit(-1.189516, 0.922747));

    m_rightLegLimits.reserve(6);
    // Hip Roll
    m_rightLegLimits.push_back(jointLimit(-0.738321, 0.414754));
    // Hip Pitch
    m_rightLegLimits.push_back(jointLimit(-1.772308, 0.485624));
    // Hip Yaw - Pitch
    m_rightLegLimits.push_back(hyp);
    // Knee Pitch
    m_rightLegLimits.push_back(jointLimit(-0.103083, 2.120198));
    // Ankle Roll
    m_rightLegLimits.push_back(jointLimit(-0.388676, 0.785875));
    // Ankle Pitch
    m_rightLegLimits.push_back(jointLimit(-1.186448, 0.932056));

    const float footWidth = m_kinematicModel->getFootInnerWidth() + m_kinematicModel->getFootOuterWidth();
    float footInnerWidth = m_kinematicModel->getFootInnerWidth();
    m_footWidth = footWidth;
    m_ballRadius = 3.5f;
    const float yReachFwd = 6.0f;
    const float yReachSide = 20.0f;
    const float xMin = m_kinematicModel->getFootForwardLength();
    const float xReachFwd = xMin + 9.0f;
    const float xReachSide = 30.0f;

    LeftFootForwardKickableArea = Rectangle(xMin, xReachFwd, (footWidth), (footWidth + yReachFwd));
    RightFootForwardKickableArea = Rectangle(xMin, xReachFwd, -(footWidth + yReachFwd), -(footWidth));

    LeftFootRightKickableArea = Rectangle(xMin, xReachSide, footWidth/2.0, yReachSide);
    //LeftFootLeftKickableArea = Rectangle(xMin, xReachSide, 2.0f*footWidth, 3.0f/2.0f*footWidth + yReachSide);
    LeftFootLeftKickableArea = Rectangle();

    RightFootLeftKickableArea = Rectangle(xMin, xReachSide, -footWidth/2.0, -yReachSide);
    //RightFootRightKickableArea = Rectangle(xMin, xReachSide, -2.0f*footWidth, -3.0f/2.0f*footWidth - yReachSide);
    RightFootRightKickableArea = Rectangle();
}

std::string NUKick::toString(legId_t theLeg)
{
    std::string result;
    switch(theLeg)
    {
    case leftLeg:
        result = "Left";
        break;
    case rightLeg:
        result = "Right";
        break;
    default:
        result = "None";
        break;
    }
    return result;
}

std::string NUKick::toString(swingDirection_t theSwingDirection)
{
    std::string result;
    switch(theSwingDirection)
    {
    case ForwardSwing:
        result = "Forward";
        break;
    case LeftSwing:
        result = "Left";
        break;
    case RightSwing:
        result = "Right";
        break;
    default:
        result = "None";
        break;
    }
    return result;
}

std::string NUKick::toString(poseType_t thePose)
{
    std::string result;
    switch(thePose)
    {
    case DO_NOTHING:
        result = "DO_NOTHING";
        break;
    case LIFT_LEG:
        result = "LIFT_LEG";
        break;
    case ADJUST_YAW:
        result = "ADJUST_YAW";
        break;
    case SET_LEG:
        result = "SET_LEG";
        break;
    case POISE_LEG:
        result = "POISE_LEG";
        break;
    case SWING:
        result = "SWING";
        break;
    case RETRACT:
        result = "RETRACT";
        break;
    case REALIGN_LEGS:
        result = "REALIGN_LEGS";
        break;
    case UNSHIFT_LEG:
        result = "UNSHIFT_LEG";
        break;
    case ALIGN_BALL:
        result = "ALIGN_BALL";
        break;
    case ALIGN_SIDE:
        result = "ALIGN_SIDE";
        break;
    case EXTEND_SIDE:
        result = "EXTEND_SIDE";
        break;
    case RESET:
        result = "RESET";
        break;
    case NO_KICK:
        result = "NO_KICK";
        break;
    case PRE_KICK:
        result = "PRE_KICK";
        break;
    case POST_KICK:
        result = "POST_KICK";
        break;
    case TRANSFER_TO_SUPPORT:
        result = "TRANSFER_TO_SUPPORT";
        break;
    default:
        result = "Unknown";
        break;
    }
    return result;
}

bool NUKick::isActive()
{
    return m_kickIsActive;
}

/*! @brief Kills the kick module
 */
void NUKick::kill()
{
    debug << "Kick kill called." << endl;
    pose = DO_NOTHING;
    m_kickIsActive = false;
    m_stateCommandGiven = false;
    m_estimatedStateCompleteTime = m_data->CurrentTime;
}

void NUKick::stop()
{
    debug << "Kick stop called." << endl;
    m_stateCommandGiven = false;
    // Chose the state that can be transitioned to allowing kick to finish as soon as possible.
    switch(pose)
    {
        case PRE_KICK:
            pose = POST_KICK;
            break;
        case TRANSFER_TO_SUPPORT:
            pose = UNSHIFT_LEG;
            break;
        case LIFT_LEG:
            pose = RETRACT;
            break;
        case POISE_LEG:
            pose = RETRACT;
        case SWING:
            pose = RETRACT;
            break;
        default:
            pose = pose;
    }
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUKick::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    m_previousTimestamp = m_currentTimestamp;
    m_currentTimestamp = data->CurrentTime;
    if(!isActive()) return;
    if (m_currentTimestamp - m_previousTimestamp > 200)
    {
        kill();
        return;
    }
    doKick();
    #ifdef USE_WALK
    if(pose == PRE_KICK)
        m_walk->process(data, actions);
    #endif // USE_WALK
}

/*! @brief Process a kick job
    @param job the kick job
 */
void NUKick::process(KickJob* job)
{
    debug << "void NUKick::process(KickJob* job)" << endl;
    double time;
    vector<float> kickposition;
    vector<float> kicktarget;
    job->getKick(time, kickposition, kicktarget);
    if(kickposition[0] == 1.0 && kickposition[1] == 2.0 && kicktarget[0] == 3.0f && kicktarget[1] == 4.0f)
        m_pauseState = !m_pauseState;
    else if(kickposition[0] == 1.0 && kickposition[1] == 1.0 && kicktarget[0] == 1.0f && kicktarget[1] == 1.0f)
    {
        m_variableGainValue += 0.001;
        debug << "Special kick command: increasing variable gain to " << m_variableGainValue << endl;
    }
    else if(kickposition[0] == -1.0 && kickposition[1] == -1.0 && kicktarget[0] == -1.0f && kicktarget[1] == -1.0f)
    {
        m_variableGainValue -= 0.001;
        debug << "Special kick command: decreasing variable gain to " << m_variableGainValue << endl;
    }
    kickToPoint(kickposition, kicktarget);
}


void NUKick::kickToPoint(const vector<float>& position, const vector<float>& target)
{
    // Evaluate the kicking target to start a new kick, or change a kick in progress.
	m_ball_x = position[0];
	m_ball_y = position[1];
	
	m_target_x = target[0];
	m_target_y = target[1];
	m_target_timestamp = nusystem->getTime();

        debug << "void NUKick::kickToPoint( (" << position[0] << "," << position[1] << "),(" << target[0] << "," << target[1] << ") )" << endl;
        debug << "current pose = " << toString(pose) << endl;

	switch(pose)
	{
            case NO_KICK:
            case DO_NOTHING:
            {
                debug << "Choosing leg." << endl;
                chooseLeg();
                return;
            }
            default:
                if(kickAbortCondition())
                {
                    stop();
                }
                break;
	}
}

void NUKick::doKick()
{
    bool done = false;

    legId_t supportLeg = noLeg;
    if(m_kickingLeg == rightLeg)
        supportLeg = leftLeg;
    else if(m_kickingLeg == leftLeg)
        supportLeg = rightLeg;
    else
    {
        pose = POST_KICK;
        return;
    }

    //currently must be at zero position
    double kickAngle = atan2(m_target_y-m_ball_y, m_target_x-m_ball_x);
    double kickDistance = sqrt(pow(m_target_y-m_ball_y,2) + pow(m_target_x-m_ball_x,2));

    debug << "void NUKick::doKick() - Current Pose: " << toString(pose) << endl;
	switch(pose)
	{
		case DO_NOTHING:
		{
			break;
		}
                case PRE_KICK:
                {
                    done = doPreKick();
                    if(done && !m_pauseState)
                    {
                        cout << "Pre kick complete!" << endl;
                        debug << "Pre kick complete!" << endl;
                        pose = TRANSFER_TO_SUPPORT;
                    }
                    break;
                }
                case POST_KICK:
                {
                    done = doPostKick();
                    if(done && !m_pauseState)
                    {
                        cout << "Post kick complete!" << endl;
                        debug << "Post kick complete!" << endl;
                        pose = DO_NOTHING;
                    }
                    break;
                }
                case TRANSFER_TO_SUPPORT:
		{
                        // Shift the weight of the robot to the support leg.
                        //done = ShiftWeightToFoot(supportLeg,1.0f,0.01, 1500);
                        done = ShiftWeightToFootClosedLoop(supportLeg, 1.0f, 0.3);
                        if(done && !m_pauseState)
                        {
                            cout << "Weight now on support foot!" << endl;
                            debug << "Weight now on support foot!" << endl;
                            pose = LIFT_LEG;
                        }
			break;
		}

		case LIFT_LEG:
		{
                        done = LiftKickingLeg(m_kickingLeg, 1.5f);
                        BalanceCoP(supportLeg);
                        if(m_swingDirection == ForwardSwing)
                        {
                            if(!m_armCommandSent)
                            {
                                MoveArmsToKickPose(m_kickingLeg, 0.7f);
                                m_armCommandSent = true;
                            }
                        }
                        if(done && !m_pauseState)
                        {
                            cout << "Leg is now lifted!" << endl;
                            debug << "Leg is now lifted!" << endl;
                            if(m_swingDirection == ForwardSwing)
                            {
                                m_armCommandSent = false;
                                pose = POISE_LEG;
                            }
                            else
                            {
                                pose = ALIGN_SIDE;
                            }
                        }
			break;
		}

		case ADJUST_YAW:
                {
                        break;
		}

		case SET_LEG:
		{
			break;
		}

		case POISE_LEG:
		{
                        done = doPoise(m_kickingLeg, 0.7, 1.5f);
                        if(m_kickingLeg == leftLeg)
                        {
                            BalanceCoP(supportLeg,0.0f,0.0f);
                        }
                        else
                        {
                            BalanceCoP(supportLeg,0.0f,0.0f);
                        }

                        if(done && !m_pauseState)
                        {
                            cout << "Leg is now poised!" << endl;
                            debug << "Leg is now poised!" << endl;
                            pose = SWING;
                        }
			break;
		}

                case ALIGN_BALL:
                {
                    done = AlignYposition(m_kickingLeg, 0.01, m_ball_y);
                    BalanceCoP(supportLeg);
                    if(done && !m_pauseState)
                    {
                        cout << "Ball is now aligned!" << endl;
                        debug << "Ball is now aligned!" << endl;
                        pose = SWING;
                    }
                   break;
                }

                case ALIGN_SIDE:
                {
                    float yTarget;
                    if(m_swingDirection == LeftSwing)
                    {
                        yTarget = m_ball_y - (m_footWidth/2.0f + m_ballRadius);
                    }
                    else
                    {
                        yTarget = m_ball_y + (m_footWidth/2.0f + m_ballRadius);
                    }
                    done = AlignYposition(m_kickingLeg, 0.01, yTarget);
                    BalanceCoP(supportLeg);
                    if(done && !m_pauseState)
                    {
                        cout << "Kicking width is now aligned!" << endl;
                        debug << "Kicking width is now aligned!" << endl;
                        pose = EXTEND_SIDE;
                    }
                   break;
                }

                case EXTEND_SIDE:
                {
                    done = AlignXposition(m_kickingLeg, 0.01, m_ball_x);
                    BalanceCoP(supportLeg);
                    if(done && !m_pauseState)
                    {
                        cout << "Kicking depth is now aligned!" << endl;
                        debug << "Kicking depth is now aligned!" << endl;
                        pose = SWING;
                    }
                   break;
                }


		case SWING:
		{
                        if(m_swingDirection == ForwardSwing)
                        {
                           debug << "Kicking Distance: " << kickDistance << endl;
                            float kickSpeed = CalculateForwardSwingSpeed(kickDistance);
                            debug << "Swinging at speed: " << kickSpeed << endl;
                            done = SwingLegForward(m_kickingLeg, kickSpeed);
                            if(!m_armCommandSent)
                            {
                                MoveArmsToKickPose(supportLeg, kickSpeed);
                                m_armCommandSent = true;
                            }
                        }
                        else if( (m_swingDirection == LeftSwing) || (m_swingDirection == RightSwing))
                        {
                            done = SwingLegSideward(m_kickingLeg, CalculateSidewardSwingSpeed(kickDistance));
                        }

                        BalanceCoP(supportLeg, 3.0f, 0.0f);
                        if(done && !m_pauseState)
                        {
                            m_armCommandSent = false;
                            cout << "Swing completed!" << endl;
                            debug << "Swing completed!" << endl;
                            pose = RETRACT;
                        }
			break;
		}
                case RETRACT:
                    {
                        done = LiftKickingLeg(m_kickingLeg, 1.0f);
                        BalanceCoP(supportLeg,3.0f,0.0f);
                        if(done && !m_pauseState)
                        {
                            cout << "Leg Retracted!" << endl;
                            debug << "Leg Retracted!" << endl;
                            pose = REALIGN_LEGS;
                        }
                        break;
                    }
                case REALIGN_LEGS:
                    {
                        done = LowerLeg(m_kickingLeg, 0.7f);
                        BalanceCoP(supportLeg,3.0f,0.0f);
                        if(done && !m_pauseState)
                        {
                            cout << "Legs Aligned!" << endl;
                            debug << "Legs Aligned!" << endl;
                            pose = UNSHIFT_LEG;
                        }
                        break;
                    }
                case UNSHIFT_LEG:
                {
                        //done = ShiftWeightToFoot(m_kickingLeg,0.5f,0.01f, 500.0f);
                        done = ShiftWeightToFootClosedLoop(supportLeg, 0.5f, 0.3);
                        if(done && !m_pauseState)
                        {
                            debug << "Weight Unshifted!" << endl;
                            pose = POST_KICK;
                        }
                        break;
                }

                case RESET:
		{
			break;
		}

		case NO_KICK:
		{
			break;
		}

		default:
		{	
			pose = NO_KICK;
			break;
		}
	}
}

bool NUKick::doPreKick()
{
    debug << "Pre - Kick" << endl;

#ifdef USE_WALK
    if(m_walk)
    {
        debug << "Walk detected - stopping robot..." << endl;
        const float maxStoppedVelocitySum = 0.4f;
        WalkJob wj(0,0,0);
        m_walk->process(&wj);
        vector<float> speed;
        m_walk->getCurrentSpeed(speed);

        if(speed.size() < 3) return false;

        bool walkStopped = allZeros(speed);

        vector<float>jointVelocities;
        float jointVelocitySum = 0.0f;
        if(m_data->getJointVelocities(NUSensorsData::BodyJoints, jointVelocities))
        {
            for (unsigned int i = 0; i < jointVelocities.size(); i++)
            {
                jointVelocitySum += fabs(jointVelocities[i]);
            }
        }
        debug << "jointVelocitySum = " << jointVelocitySum << endl;
        debug << "Walk stopped = " << walkStopped << endl;
        bool robotStopped = walkStopped && (jointVelocitySum < maxStoppedVelocitySum);
        debug << "Robot stopped = " << robotStopped << endl;
        if(!robotStopped)
            return false;
    }
#endif // USE_WALK
    bool validData = true;
    vector<float>leftJoints;
    vector<float>rightJoints;
    validData = validData && m_data->getJointPositions(NUSensorsData::LeftLegJoints,leftJoints);
    validData = validData && m_data->getJointPositions(NUSensorsData::RightLegJoints,rightJoints);
    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);

    vector<float> leftArmJoints;
    vector<float> rightArmJoints;
    validData = validData && m_data->getJointPositions(NUSensorsData::LeftArmJoints,leftArmJoints);
    validData = validData && m_data->getJointPositions(NUSensorsData::RightArmJoints,rightArmJoints);

    if(!m_stateCommandGiven && validData)
    {
        debug << "Moving to Initial Position" << endl;
    #ifdef USE_WALK
        m_walk->kill();
    #endif // USE_WALK
        vector<float> armpos (4, 0.0f);
        armpos[1] = PI/2.0f;
        float maxSpeed = 0.35;
        double leftTime,rightTime;
        leftTime = MoveLimbToPositionWithSpeed(NUActionatorsData::LeftLegJoints, leftJoints, m_leftLegInitialPose, maxSpeed , 75.0);
        rightTime = MoveLimbToPositionWithSpeed(NUActionatorsData::RightLegJoints, rightJoints, m_rightLegInitialPose, maxSpeed , 75.0);
        armpos[0] = PI/8.0;
        armpos[3] = -PI/2.0f;
        MoveLimbToPositionWithSpeed(NUActionatorsData::LeftArmJoints, leftArmJoints, armpos, 2*maxSpeed , 40.0);
        armpos[0] = -PI/8.0;
        armpos[3] = PI/2.0f;
        MoveLimbToPositionWithSpeed(NUActionatorsData::RightArmJoints, rightArmJoints, armpos, 2*maxSpeed , 40.0);
        m_stateCommandGiven = true;
        m_estimatedStateCompleteTime = max(rightTime,leftTime);
    }
    if(validData)
    {
        if((allEqual(m_leftLegInitialPose, leftJoints, 0.05f) && allEqual(m_leftLegInitialPose, rightJoints, 0.05f)) || (m_data->CurrentTime - m_estimatedStateCompleteTime > 200.0))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}

bool NUKick::doPostKick()
{
    m_kickIsActive = false;
    m_kickingLeg = noLeg;
    return true;
}

bool NUKick::ShiftWeightToFoot(legId_t targetLeg, float targetWeightPercentage, float speed, float time)
{
    const float maxShiftSpeed = speed * SpeedMultiplier();
    const float reqiredAccuracy = 0.1; // 10%
    const float propGain = 0.05f;
    bool validData = true;
    bool leftContact, rightContact;
    validData = validData && m_data->getFootContact(NUSensorsData::LeftFoot, leftContact);
    validData = validData && m_data->getFootContact(NUSensorsData::RightFoot, rightContact);
    if(!validData) return false;
    float lcopx(0.0f),lcopy(0.0f),rcopx(0.0f),rcopy(0.0f);
    float lforce(0.0f),rforce(0.0f);

    float leftImpactTime, rightImpactTime;
    bool recentLeftImpact = m_data->footImpact(NUSensorsData::LeftFoot,leftImpactTime);
    bool recentRightImpact = m_data->footImpact(NUSensorsData::RightFoot,rightImpactTime);

    if(leftContact)
    {
        validData = validData && m_data->getFootCoP(NUSensorsData::LeftFoot,lcopx,lcopy);
        validData = validData && m_data->getFootForce(NUSensorsData::LeftFoot,lforce);
    }
    if(rightContact)
    {
        validData = validData && m_data->getFootCoP(NUSensorsData::RightFoot,rcopx,rcopy);
        validData = validData && m_data->getFootForce(NUSensorsData::RightFoot,rforce);
    }



    vector<float>leftJoints;
    vector<float>rightJoints;
    validData = validData && m_data->getJointTargets(NUSensorsData::LeftLegJoints,leftJoints);
    validData = validData && m_data->getJointTargets(NUSensorsData::RightLegJoints,rightJoints);
    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, m_defaultMotorGain);
        float weightPercentage;
        float weightError = 0;
        bool recentImpact(false);
        float newHipPos(0.0f);
        if(targetLeg == rightLeg)
        {
            weightPercentage = rforce / (rforce + lforce);
            weightError = targetWeightPercentage - weightPercentage;
            recentImpact = recentRightImpact;
            recentImpact = m_data->CurrentTime - rightImpactTime < 5*TimeBetweenFrames();
            newHipPos = rightJoints[0] + crop(weightError*propGain, -maxShiftSpeed, maxShiftSpeed);
        }
        else if(targetLeg == leftLeg)
        {
            weightPercentage = lforce / (rforce + lforce);
            weightError = weightPercentage - targetWeightPercentage;
            recentImpact = recentLeftImpact;
            recentImpact = m_data->CurrentTime - leftImpactTime < 5*TimeBetweenFrames();
            newHipPos = leftJoints[0] + crop(weightError*propGain, -maxShiftSpeed, maxShiftSpeed);
        }
        if(!m_stateCommandGiven)
        {
            m_stateCommandGiven = true;
            m_estimatedStateCompleteTime = m_data->CurrentTime + time;
        }
        if(fabs(weightError) > reqiredAccuracy)
        {
            leftJoints[0] = newHipPos;
            rightJoints[0] = newHipPos;
            leftJoints[4] = -newHipPos;
            rightJoints[4] = -newHipPos;
            m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
            m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
        }
        else
        {
            BalanceCoP(targetLeg);
        }
        //float elapsedTime = m_data->CurrentTime - (m_estimatedStateCompleteTime - time);
        if(m_stateCommandGiven && !recentImpact && (m_estimatedStateCompleteTime < m_data->CurrentTime))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}


bool NUKick::ShiftWeightToFootClosedLoop(legId_t targetLeg, float targetWeightPercentage, float speed)
{
    bool validData = true;
    NUSensorsData::bodypart_id_t s_targetLeg;
    NUSensorsData::bodypart_id_t s_otherLeg;
    NUActionatorsData::bodypart_id_t a_targetLeg;
    NUActionatorsData::bodypart_id_t a_otherLeg;
    float targetDisplacement;
    if(targetLeg == rightLeg)
    {
        s_targetLeg = NUSensorsData::RightLegJoints;
        s_otherLeg = NUSensorsData::LeftLegJoints;
        a_targetLeg = NUActionatorsData::RightLegJoints;
        a_otherLeg = NUActionatorsData::LeftLegJoints;
        targetDisplacement = 2 * (0.5 - targetWeightPercentage) * m_kinematicModel->getHipOffsetY();
    }
    else if(targetLeg == leftLeg)
    {
        s_targetLeg = NUSensorsData::LeftLegJoints;
        s_otherLeg = NUSensorsData::RightLegJoints;
        a_targetLeg = NUActionatorsData::LeftLegJoints;
        a_otherLeg = NUActionatorsData::RightLegJoints;
        targetDisplacement = 2 * (targetWeightPercentage - 0.5) * m_kinematicModel->getHipOffsetY();
    }
    else return true;

    vector<float>targetLegPositions;
    vector<float>otherLegPositions;
    validData = validData && m_data->getJointPositions(s_targetLeg,targetLegPositions);
    validData = validData && m_data->getJointPositions(s_otherLeg,otherLegPositions);

    validData = validData && (targetLegPositions.size() >= 6) && (otherLegPositions.size() >= 6);

    static vector<float> legPositionTargets;

    if(validData)
    {
        if(!m_stateCommandGiven)
        {

            float targetLegLength = m_kinematicModel->CalculateRadialLegLength(targetLegPositions);
            float otherLegLength = m_kinematicModel->CalculateRadialLegLength(otherLegPositions);
            float targetLength;
            if(targetLegLength < otherLegLength)
            {
                targetLength = targetLegLength;
                legPositionTargets = targetLegPositions;
            }
            else
            {
                targetLength = otherLegLength;
                legPositionTargets = otherLegPositions;
            }

            debug << "targetDisplacement = " << targetDisplacement << endl;
            debug << "targetLength = " << targetLength << endl;
            float targetAnkleRoll = asin(targetDisplacement / targetLength);
            debug << "targetAnkleRoll = " << targetAnkleRoll << endl;
            legPositionTargets[0] = -targetAnkleRoll;
            legPositionTargets[4] = targetAnkleRoll;
            MoveLimbToPositionWithSpeed(a_targetLeg,targetLegPositions,legPositionTargets,speed,m_defaultMotorGain,1.0);
            m_estimatedStateCompleteTime = MoveLimbToPositionWithSpeed(a_otherLeg,otherLegPositions,legPositionTargets,speed,m_defaultMotorGain,1.0);
            debug << "Sending move command" << endl;
            m_stateCommandGiven = true;
        }

        if((allEqual(targetLegPositions, legPositionTargets, 0.05f) && allEqual(otherLegPositions, legPositionTargets, 0.05f)) || (m_data->CurrentTime - m_estimatedStateCompleteTime > 200.0))
        {
            if((m_data->CurrentTime - m_estimatedStateCompleteTime > 200.0))
            {
                debug << "State timed out" << endl;
            }
            else
            {
                debug << "targetLegPositions[4] = " << targetLegPositions[4] << endl;
                debug << "otherLegPositions[4] = " << otherLegPositions[4] << endl;
                debug << "legPositionTargets[4] = " << legPositionTargets[4] << endl;
                debug << "Targets Reached" << endl;
            }
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}


bool NUKick::LiftKickingLeg(legId_t kickingLeg, float speed)
{
    bool validData = true;
    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(kickingLeg == rightLeg)
    {
        s_kickingFoot = NUSensorsData::RightFoot;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
        s_supportLeg = NUSensorsData::LeftLegJoints;
    }
    else if(kickingLeg == leftLeg)
    {
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
        s_supportLeg = NUSensorsData::RightLegJoints;
    }
    else return true;

    bool kickContact;
    validData = validData && m_data->getFootContact(s_kickingFoot, kickContact);
    if(!validData) return false;

    vector<float>kickLegJoints;
    vector<float>kickLegPositions;
    vector<float>supportLegPositions;
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickLegJoints);
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickLegPositions);
    validData = validData && m_data->getJointPositions(s_supportLeg,supportLegPositions);

    validData = validData && (kickLegJoints.size() >= 6);
    static vector<float>kickLegTargets;
    if(validData)
    {
        if(!m_stateCommandGiven)
        {
            kickLegTargets = supportLegPositions;
            kickLegTargets[3] = 1.6f;
            kickLegTargets[1] = -kickLegTargets[3] / 2.0f;
            kickLegTargets[5] = -kickLegTargets[3] / 2.0f;
            debug << "Motion Command Given." << endl;
            m_estimatedStateCompleteTime = MoveLimbToPositionWithSpeed(a_kickingLeg, kickLegPositions, kickLegTargets, speed, 75.0);
            m_stateCommandGiven = true;
        }

        if(allEqual(kickLegTargets, kickLegPositions, 0.05f) || (m_data->CurrentTime - m_estimatedStateCompleteTime > 200.0))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}

bool NUKick::doPoise(legId_t poiseLeg, float angleChange, float speed)
{
    bool validData = true;
    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(poiseLeg == rightLeg)
    {
        s_kickingFoot = NUSensorsData::RightFoot;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
    }
    else if(poiseLeg == leftLeg)
    {
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
    }
    else return true;

    bool kickContact;
    validData = validData && m_data->getFootContact(s_kickingFoot, kickContact);
    if(!validData) return false;

    vector<float>kickLegJoints;
    vector<float>kickLegPositions;
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickLegJoints);
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickLegPositions);
    validData = validData && (kickLegJoints.size() >= 6) && (kickLegPositions.size() >= 6);

    static vector<float>kickLegTargets;

    if(validData)
    {
        if(!m_stateCommandGiven)
        {
            kickLegTargets.assign(kickLegPositions.begin(), kickLegPositions.end());
            kickLegTargets[1] = kickLegPositions[1] + angleChange;
            FlattenFoot(kickLegTargets);
            LimitJoints(poiseLeg, kickLegTargets);
            debug << "Motion Command Given." << endl;
            m_estimatedStateCompleteTime = MoveLimbToPositionWithSpeed(a_kickingLeg, kickLegPositions, kickLegTargets, speed , m_defaultMotorGain);
            m_stateCommandGiven = true;
        }
        if(allEqual(kickLegTargets, kickLegPositions, 0.05f) || (m_data->CurrentTime - m_estimatedStateCompleteTime > 200.0))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}

double NUKick::TimeBetweenFrames()
{
    return (m_currentTimestamp - m_previousTimestamp);
}

float NUKick::perSec2perFrame(float value)
{
    return value * (TimeBetweenFrames() / 1000.0);
}

float NUKick::SpeedMultiplier()
{
    return TimeBetweenFrames() / 20.0f;
}

float NUKick::GainMultiplier()
{
    return TimeBetweenFrames() / 20.0f;
}

bool NUKick::LimitJoints(legId_t leg, vector<float> jointPositions)
{
    bool changed = false;
    float previousValue;
    if(leg == rightLeg)
    {
        for(unsigned int i = 0; i < jointPositions.size(); i++)
        {
            previousValue = jointPositions[i];
            jointPositions[i] = crop(previousValue, m_rightLegLimits[i].min, m_rightLegLimits[i].max);
            changed = changed || (jointPositions[i] != previousValue);
        }
    }
    else if (leg == leftLeg)
    {
        for(unsigned int i = 0; i < jointPositions.size(); i++)
        {
            previousValue = jointPositions[i];
            jointPositions[i] = crop(previousValue, m_leftLegLimits[i].min, m_leftLegLimits[i].max);
            changed = changed || (jointPositions[i] != previousValue);
        }
    }
    return changed;
}


bool NUKick::IsPastTime(float time){
    return (m_data->CurrentTime > time);
}

bool NUKick::BalanceCoP(legId_t supportLeg, float targetX, float targetY)
{
    bool validData = true;

    NUSensorsData::foot_id_t s_supportFoot;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUActionatorsData::bodypart_id_t a_supportLeg;
    if(supportLeg == rightLeg)
    {
        s_supportFoot = NUSensorsData::RightFoot;
        s_supportLeg = NUSensorsData::RightLegJoints;
        a_supportLeg = NUActionatorsData::RightLegJoints;
    }
    else if(supportLeg == leftLeg)
    {
        s_supportFoot = NUSensorsData::LeftFoot;
        s_supportLeg = NUSensorsData::LeftLegJoints;
        a_supportLeg = NUActionatorsData::LeftLegJoints;
    }
    else return true;

    bool supportFootContact, isSupport;
    validData = validData && m_data->getFootContact(s_supportFoot, supportFootContact);
    validData = validData && m_data->getFootSupport(s_supportFoot,isSupport);
    if(validData && supportFootContact)
    {
        vector<float>supportLegJoints;
        validData = validData && m_data->getJointTargets(s_supportLeg,supportLegJoints);
        validData = validData && (supportLegJoints.size() >= 6);
        float copx(0.0f),copy(0.0f);
        float force(0.0f);
        validData = validData && m_data->getFootCoP(s_supportFoot,copx,copy);
        validData = validData && m_data->getFootForce(s_supportFoot,force);
        if(validData)
        {
            BalanceCoPLevelTorso(supportLegJoints, copx, copy, targetX, targetY);
            //BalanceCoPHipAndAnkle(supportLegJoints, copx, copy);
            LimitJoints(supportLeg,supportLegJoints);
            vector<float> vel (6, 0);
            vector<float> gain (6, m_defaultMotorGain);
            m_actions->addJointPositions(a_supportLeg, m_data->CurrentTime, supportLegJoints, vel, gain);
            return true;
        }
    }
    return false;
}

void NUKick::BalanceCoPLevelTorso(vector<float>& jointAngles, float CoPx, float CoPy, float targetX, float targetY)
{
    // Linear controller to centre CoP
    const float gainx = -0.01 * GainMultiplier();
    const float gainy = 0.006 * GainMultiplier();
    const float targetCoPx = targetX;
    const float targetCoPy = targetY;

    const float deltax = targetCoPx - CoPx;
    const float deltay = targetCoPy - CoPy;
//    const float gainx = m_variableGainValue * GainMultiplier();
//    const float gainy = m_variableGainValue * GainMultiplier();

    // Roll correction
    // Ankle Roll
    jointAngles[4] += gainy * asin(deltay / 35.0);
    // Hip Roll - Reverse of pitch to maintain vertical torso.
    jointAngles[0] = -jointAngles[4];

    // Pitch correction
    jointAngles[5] += gainx * asin(deltax / 35.0f);
    // Hip Pitch - Reverse of pitch to maintain vertical torso.
    jointAngles[2] = 0.5*jointAngles[5];
    return;
}

void NUKick::BalanceCoPHipAndAnkle(vector<float>& jointAngles, float CoPx, float CoPy, float targetX, float targetY)
{
    // Linear controller to centre CoP
//    const float gainx = 0.008 * GainMultiplier();
//    const float gainy = 0.008 * GainMultiplier();
    const float gainx = m_variableGainValue * GainMultiplier();
    const float gainy = m_variableGainValue * GainMultiplier();

    // Roll correction
    // Ankle Roll
    jointAngles[4] -= gainy * asin(CoPy / 35.0);
    // Hip Roll - Reverse of pitch to maintain vertical torso.
    jointAngles[0] -= 10 * gainy * asin(CoPy / 35.0);

    // Pitch correction
    jointAngles[5] -= gainx * asin(CoPx / 35.0f);
    // Hip Pitch - Reverse of pitch to maintain vertical torso.
    jointAngles[2] -= 10 * gainx * asin(CoPx / 35.0f);
    return;
}

bool NUKick::AlignXposition(legId_t kickingLeg, float speed, float xPos)
{
    const float gain = 0.01;
    bool validData = true;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    NUActionatorsData::joint_id_t a_kickingHipPitch;
    NUActionatorsData::joint_id_t a_kickingKneePitch;
    NUActionatorsData::joint_id_t a_kickingAnklePitch;
    Kinematics::Effector k_kickingLeg;
    jointLimit hipPitchJointLimits;
    jointLimit kneePitchJointLimits;
    jointLimit anklePitchJointLimits;

    Matrix supportLegTransform, kickingLegTransform;
    m_data->getSupportLegTransform(supportLegTransform);
    if(kickingLeg == rightLeg)
    {
        debug << "Aligning with right leg" << endl;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
        a_kickingHipPitch = NUActionatorsData::RHipPitch;
        a_kickingKneePitch = NUActionatorsData::RKneePitch;
        a_kickingAnklePitch = NUActionatorsData::RAnklePitch;
        validData = validData && m_data->getRightLegTransform(kickingLegTransform);
        hipPitchJointLimits = m_rightLegLimits[1];
        kneePitchJointLimits = m_rightLegLimits[3];
        anklePitchJointLimits = m_rightLegLimits[5];
        k_kickingLeg = Kinematics::rightFoot;
    }
    else if(kickingLeg == leftLeg)
    {
        debug << "Aligning with left leg" << endl;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
        a_kickingHipPitch = NUActionatorsData::LHipPitch;
        a_kickingKneePitch = NUActionatorsData::LKneePitch;
        a_kickingAnklePitch = NUActionatorsData::LAnklePitch;
        k_kickingLeg = Kinematics::leftFoot;
        validData = validData && m_data->getLeftLegTransform(kickingLegTransform);
        hipPitchJointLimits = m_leftLegLimits[1];
        kneePitchJointLimits = m_leftLegLimits[3];
        anklePitchJointLimits = m_leftLegLimits[5];
    }
    else return true;

    vector<float>kickLegJoints;
    vector<float>kickLegPositions;
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickLegJoints);
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickLegPositions);
    validData = validData && (kickLegJoints.size() >= 6);
    vector<float> footPosition = Kinematics::PositionFromTransform(kickingLegTransform);
    float deltaX = xPos - footPosition[0];
    debug << "Delta X = " << deltaX;
    bool jointLimitReached = false;
    if(validData)
    {
        const float targetHeight = 5.0f;

        float currentHeightOffGround = m_kinematicModel->CalculateRelativeFootHeight(supportLegTransform,kickingLegTransform,k_kickingLeg);
        debug << "X Position = " << xPos << "foot Position = (" << footPosition[0] << "," << footPosition[1] << "," << footPosition[2] << ")" << endl;
        debug << "Calculated height of foot from ground  = " << currentHeightOffGround << endl;
        float deltaTheta = crop(gain*deltaX,-speed,speed);
        float calcKneePitchAngle = kickLegJoints[3] - 2*deltaTheta;

        float calcHipPitchAngle = kickLegJoints[1] - crop(gain*(targetHeight-currentHeightOffGround),-speed,speed);

        float calcAnklePitchAngle = FlatFootAnklePitch(calcHipPitchAngle,calcKneePitchAngle);
        float newHipPitchAngle = crop(calcHipPitchAngle,hipPitchJointLimits.min,hipPitchJointLimits.max);
        float newKneePitchAngle = crop(calcKneePitchAngle,kneePitchJointLimits.min,kneePitchJointLimits.max);
        float newAnklePitchAngle = crop(calcAnklePitchAngle,anklePitchJointLimits.min,anklePitchJointLimits.max);
        m_actions->addJointPosition(a_kickingHipPitch,m_data->CurrentTime,newHipPitchAngle,0,m_defaultMotorGain);
        m_actions->addJointPosition(a_kickingKneePitch,m_data->CurrentTime,newKneePitchAngle,0,m_defaultMotorGain);
        m_actions->addJointPosition(a_kickingAnklePitch,m_data->CurrentTime,newAnklePitchAngle,0,m_defaultMotorGain);
        jointLimitReached = (newHipPitchAngle != calcHipPitchAngle) || (newKneePitchAngle != calcKneePitchAngle) || (newAnklePitchAngle != calcAnklePitchAngle);
    }
    return (fabs(deltaX) < 0.5) || jointLimitReached;
}

bool NUKick::AlignYposition(legId_t kickingLeg, float speed, float yPos)
{
    const float gain = 0.01;
    bool validData = true;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    NUActionatorsData::joint_id_t a_kickingHipRoll;
    Matrix kickingLegTransform;
    jointLimit hipJointLimits;
    if(kickingLeg == rightLeg)
    {
        debug << "Aligning with right leg" << endl;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
        a_kickingHipRoll = NUActionatorsData::RHipRoll;
        validData = validData && m_data->getRightLegTransform(kickingLegTransform);
        hipJointLimits = m_rightLegLimits[0];
    }
    else if(kickingLeg == leftLeg)
    {
        debug << "Aligning with left leg" << endl;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
        a_kickingHipRoll = NUActionatorsData::LHipRoll;
        validData = validData && m_data->getLeftLegTransform(kickingLegTransform);
        hipJointLimits = m_leftLegLimits[0];
    }
    else return true;

    vector<float>kickLegJoints;
    vector<float>kickLegPositions;
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickLegJoints);
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickLegPositions);
    validData = validData && (kickLegJoints.size() >= 6);
    vector<float> footPosition = Kinematics::PositionFromTransform(kickingLegTransform);
    float deltaY = yPos - footPosition[1];
    debug << "Delta Y = " << deltaY;
    bool jointLimitReached = false;
    if(validData)
    {
        debug << "Y Position = " << yPos << "foot Position = (" << footPosition[0] << "," << footPosition[1] << "," << footPosition[2] << ")" << endl;
        float deltaTheta = crop(gain*deltaY,-speed,speed);
        float calcHipRollAngle = kickLegJoints[0] + deltaTheta;
        float newHipRollAngle = crop(calcHipRollAngle,hipJointLimits.min,hipJointLimits.max);
        m_actions->addJointPosition(a_kickingHipRoll,m_data->CurrentTime,newHipRollAngle,0,m_defaultMotorGain);
        jointLimitReached = (newHipRollAngle != calcHipRollAngle);
    }
    return (fabs(deltaY) < 0.5) || jointLimitReached;
}

void NUKick::FlattenFoot(vector<float>& jointAngles)
{
    jointAngles[5] = FlatFootAnklePitch(jointAngles[1], jointAngles[3]);
    jointAngles[4] = FlatFootAnkleRoll(jointAngles[0]);
    return;
}

float NUKick::FlatFootAnklePitch(float hipPitch, float kneePitch)
{
    return -(hipPitch + kneePitch);
}

float NUKick::FlatFootAnkleRoll(float hipRoll)
{
    return -(hipRoll);
}

void NUKick::MaintainSwingHeight(legId_t supportLeg, vector<float>& supportLegJoints, legId_t swingLeg, vector<float>& swingLegJoints, float swingHeight)
{
    return;
}

void NUKick::MoveArmsToKickPose(legId_t leadingArmleg, float speed)
{
    NUActionatorsData::bodypart_id_t a_leadingArm;
    NUSensorsData::bodypart_id_t s_leadingArm;
    NUActionatorsData::bodypart_id_t a_trailingArm;
    NUSensorsData::bodypart_id_t s_trailingArm;
    float mirroredJointMultiplier = 1.0;
    if(leadingArmleg == rightLeg)
    {
        a_leadingArm = NUActionatorsData::RightArmJoints;
        s_leadingArm = NUSensorsData::RightArmJoints;
        a_trailingArm = NUActionatorsData::LeftArmJoints;
        s_trailingArm = NUSensorsData::LeftArmJoints;
    }
    else if(leadingArmleg == leftLeg)
    {
        a_leadingArm = NUActionatorsData::LeftArmJoints;
        s_leadingArm = NUSensorsData::LeftArmJoints;
        a_trailingArm = NUActionatorsData::RightArmJoints;
        s_trailingArm = NUSensorsData::RightArmJoints;
        mirroredJointMultiplier *= -1.0f;
    }
    else return;

    bool validData = true;
    vector<float> leadingArmPositions,trailingArmPositions;
    validData = validData && m_data->getJointPositions(s_leadingArm,leadingArmPositions);
    validData = validData && m_data->getJointPositions(s_trailingArm,trailingArmPositions);

    if(validData)
    {
        vector<float> leadingArmTargets(leadingArmPositions),trailingArmTargets(trailingArmPositions);
        // Shoulder Yaw
        leadingArmTargets[0] = -deg2rad(35.0) * mirroredJointMultiplier;
        trailingArmTargets[0] = deg2rad(35.0) * mirroredJointMultiplier;

        // Shoulder Pitch
        leadingArmTargets[1] = PI/4.0f;
        trailingArmTargets[1] = 2.1f;

        // Elbow Yaw
        leadingArmTargets[2] = deg2rad(70.0) * mirroredJointMultiplier;
        trailingArmTargets[2] = -deg2rad(35.0) * mirroredJointMultiplier;

        // Elbow Roll
        leadingArmTargets[3] = 0.0f  * mirroredJointMultiplier;
        trailingArmTargets[3] = 0.0f * mirroredJointMultiplier;



        MoveLimbToPositionWithSpeed(a_leadingArm, leadingArmPositions, leadingArmTargets, speed , 75.0);
        MoveLimbToPositionWithSpeed(a_trailingArm, trailingArmPositions, trailingArmTargets, speed , 75.0);
    }
}

bool NUKick::SwingLegForward(legId_t kickingLeg, float speed)
{
    float swingSpeed = speed;
    bool validData = true;

    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    NUActionatorsData::joint_id_t a_kickingKneePitch;
    NUActionatorsData::joint_id_t a_kickingHipPitch;
    NUActionatorsData::joint_id_t a_kickingAnklePitch;
    if(kickingLeg == rightLeg)
    {
        s_kickingFoot = NUSensorsData::RightFoot;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
        a_kickingHipPitch = NUActionatorsData::RHipPitch;
        a_kickingKneePitch = NUActionatorsData::RKneePitch;
        a_kickingAnklePitch = NUActionatorsData::RAnklePitch;
    }
    else if(kickingLeg == leftLeg)
    {
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
        a_kickingHipPitch = NUActionatorsData::LHipPitch;
        a_kickingKneePitch = NUActionatorsData::LKneePitch;
        a_kickingAnklePitch = NUActionatorsData::LAnklePitch;
    }
    else return true;

    if(!validData) return false;

    vector<float>kickingLegJoints;
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickingLegJoints);
    validData = validData && (kickingLegJoints.size() >= 6);

    const float targetHipPitch = -1.0;
    const float targetKneePitch = 0.8f;

    static float endHipAngle = 0;
    static float endKneeAngle = 0;
    static float endAnkleAngle = 0;

    if(validData)
    {

        if(m_stateCommandGiven && (endHipAngle-kickingLegJoints[1] >= -0.05) && (endKneeAngle-kickingLegJoints[3] >= -0.05))
        {
            m_stateCommandGiven = false;
            return true;
        }

        if(!m_stateCommandGiven)
        {
            float startHipAngle = kickingLegJoints[1];
            float startKneeAngle = kickingLegJoints[3];
            float startAnkleAngle = -(startHipAngle + startKneeAngle);

            endHipAngle = targetHipPitch;//startHipAngle - PI/4;
            endKneeAngle = targetKneePitch;//startKneeAngle - PI/4;
            endAnkleAngle = -(endHipAngle + endKneeAngle);

            vector<double> hipTimes, kneeTimes, ankleTimes;
            vector<float> hipPositions, hipVelocities, kneePositions, kneeVelocities, anklePositions, ankleVelocities;
            vector<float> kickingTargets(kickingLegJoints);
            kickingTargets[1] = endHipAngle;
            kickingTargets[3] = endKneeAngle;
            FlattenFoot(kickingTargets);

            /*
            const float startTime = m_data->CurrentTime+100.0;
            const float maxChange = max(max(fabs(endHipAngle - startHipAngle),fabs(endKneeAngle - startKneeAngle)),fabs(endAnkleAngle-startAnkleAngle));
            const float swingTime = 1000.0*maxChange / swingSpeed;
            const float endTime = startTime + swingTime;
            debug << "staring swing command - Estimated swing time = " << swingTime << endl;
            MotionCurves::calculate(startTime,endTime, startHipAngle, endHipAngle, 1.0, 20.0f, hipTimes, hipPositions, hipVelocities);
            MotionCurves::calculate(startTime,endTime, startKneeAngle, endKneeAngle, 1.0, 20.0f, kneeTimes, kneePositions, kneeVelocities);
            MotionCurves::calculate(startTime,endTime, startAnkleAngle, endAnkleAngle, 1.0, 20.0f, ankleTimes, anklePositions, ankleVelocities);
            m_actions->addJointPositions(a_kickingHipPitch,hipTimes,hipPositions,hipVelocities,100.0);
            m_actions->addJointPositions(a_kickingKneePitch,kneeTimes,kneePositions,kneeVelocities,100.0);
            m_actions->addJointPositions(a_kickingAnklePitch,ankleTimes,anklePositions,ankleVelocities,100.0);
            */
            MoveLimbToPositionWithSpeed(a_kickingLeg,kickingLegJoints,kickingTargets,swingSpeed,100.0f,1.0f);
            m_stateCommandGiven = true;
        }
    }
    return false;
}

bool NUKick::SwingLegSideward(legId_t kickingLeg, float speed)
{
    float swingSpeed = speed;
    bool validData = true;

    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(kickingLeg == rightLeg)
    {
        s_kickingFoot = NUSensorsData::RightFoot;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        s_supportLeg = NUSensorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
    }
    else if(kickingLeg == leftLeg)
    {
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        s_supportLeg = NUSensorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
    }
    else return true;

    if(!validData) return false;

    vector<float>kickingLegJoints, supportLegJoints;
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickingLegJoints);
    validData = validData && m_data->getJointPositions(s_supportLeg,supportLegJoints);
    validData = validData && (kickingLegJoints.size() >= 6);

    static vector<float> swingTargets;

    if(validData)
    {
        if(!m_stateCommandGiven)
        {
            swingTargets = supportLegJoints;

            swingTargets[1] = kickingLegJoints[1] - 0.125;
            swingTargets[3] = kickingLegJoints[3] + 0.25;
            FlattenFoot(swingTargets);

            m_estimatedStateCompleteTime = MoveLimbToPositionWithSpeed(a_kickingLeg, kickingLegJoints, swingTargets, swingSpeed , 100.0);
            m_stateCommandGiven = true;
        }
        if(allEqual(swingTargets, kickingLegJoints, 0.05f) || (m_data->CurrentTime - m_estimatedStateCompleteTime > 200.0))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}

bool NUKick::LowerLeg(legId_t kickingLeg, float speed)
{
    bool validData = true;
    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(kickingLeg == rightLeg)
    {
        s_kickingFoot = NUSensorsData::RightFoot;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
        s_supportLeg = NUSensorsData::LeftLegJoints;
    }
    else if(kickingLeg == leftLeg)
    {
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
        s_supportLeg = NUSensorsData::RightLegJoints;
    }
    else return true;

    bool kickContact;
    validData = validData && m_data->getFootContact(s_kickingFoot, kickContact);
    if(!validData) return false;

    vector<float>kickLegJoints;
    vector<float>kickLegPositions;
    vector<float>supportLegPositions;
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickLegJoints);
    validData = validData && m_data->getJointPositions(s_kickingLeg,kickLegPositions);
    validData = validData && m_data->getJointTargets(s_supportLeg,supportLegPositions);

    validData = validData && (kickLegJoints.size() >= 6);

    static vector<float>kickLegTargets;

    if(validData)
    {
        if(!m_stateCommandGiven)
        {
            kickLegTargets = supportLegPositions;
            debug << "Motion Command Given." << endl;
            m_estimatedStateCompleteTime = MoveLimbToPositionWithSpeed(a_kickingLeg, kickLegPositions, kickLegTargets, speed, 75.0, 1.0f);
            m_stateCommandGiven = true;
        }

        if(allEqual(kickLegTargets, kickLegPositions, 0.05f) || (m_data->CurrentTime - m_estimatedStateCompleteTime > 200.0))
        {
            m_stateCommandGiven = false;
            return true;
        }
    }
    return false;
}

//bool NUKick::LowerLeg(legId_t kickingLeg)
//{
//    const float maxSpeed = 0.01 * SpeedMultiplier();
//    const float pGain = 0.1;
//    bool validData = true;
//    bool leftContact, rightContact;
//    validData = validData && m_data->getFootContact(NUSensorsData::LeftFoot, leftContact);
//    validData = validData && m_data->getFootContact(NUSensorsData::RightFoot, rightContact);
//    if(!validData) return false;
//    float lcopx(0.0f),lcopy(0.0f),rcopx(0.0f),rcopy(0.0f);
//    float lforce(0.0f),rforce(0.0f);
//
//    if(leftContact)
//    {
//        validData = validData && m_data->getFootCoP(NUSensorsData::LeftFoot,lcopx,lcopy);
//        validData = validData && m_data->getFootForce(NUSensorsData::LeftFoot,lforce);
//    }
//    if(rightContact)
//    {
//        validData = validData && m_data->getFootCoP(NUSensorsData::RightFoot,rcopx,rcopy);
//        validData = validData && m_data->getFootForce(NUSensorsData::RightFoot,rforce);
//    }
//
//    vector<float>leftJoints;
//    vector<float>rightJoints;
//    validData = validData && m_data->getJointTargets(NUSensorsData::LeftLegJoints,leftJoints);
//    validData = validData && m_data->getJointTargets(NUSensorsData::RightLegJoints,rightJoints);
//    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);
//
//    if(validData)
//    {
//        vector<float> vel (6, 0);
//        vector<float> gain (6, m_defaultMotorGain);
//
//        if(kickingLeg == rightLeg)
//        {
//            BalanceCoPLevelTorso(leftJoints, lcopx, lcopy);
//            rightJoints[0] = leftJoints[0];
//            rightJoints[4] = leftJoints[4];
//
//            const float targetHipPitch = leftJoints[1];
//            const float targetKneePitch = leftJoints[3];
//
//            float hipDiff = targetHipPitch - rightJoints[1];
//            float kneeDiff = targetKneePitch - rightJoints[3];
//            if((fabs(hipDiff) + fabs(kneeDiff)) < 0.1) return true;
//
//            rightJoints[1] += crop(pGain*hipDiff,-maxSpeed,maxSpeed);
//            rightJoints[3] += crop(pGain*kneeDiff,-maxSpeed,maxSpeed);
//            FlattenFoot(rightJoints);
//        }
//        else if(kickingLeg == leftLeg)
//        {
//            BalanceCoPLevelTorso(rightJoints, lcopx, lcopy);
//            leftJoints[0] = rightJoints[0];
//            leftJoints[4] = rightJoints[4];
//
//            const float targetHipPitch = rightJoints[1];
//            const float targetKneePitch = rightJoints[3];
//
//            float hipDiff = targetHipPitch - leftJoints[1];
//            float kneeDiff = targetKneePitch - leftJoints[3];
//            if( ((fabs(hipDiff) + fabs(kneeDiff)) < 0.1) && (leftContact && rightContact))return true;
//
//            leftJoints[1] += crop(pGain*hipDiff,-maxSpeed,maxSpeed);
//            leftJoints[3] += crop(pGain*kneeDiff,-maxSpeed,maxSpeed);
//            FlattenFoot(leftJoints);
//        }
//        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
//        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
//    }
//    return false;
//}

bool NUKick::kickAbortCondition()
{
    return false;
}

float NUKick::CalculateForwardSwingSpeed(float kickDistance)
{
    return 30.0f;
}

float NUKick::CalculateSidewardSwingSpeed(float kickDistance)
{
    return 4.0f;
}

bool NUKick::chooseLeg()
{
    //currently must be at zero position
	double theta = atan2(m_target_y-m_ball_y, m_target_x-m_ball_x);
	
	//approximate, assume robot torso moves negligible amount and only rotates
	double xtrans = m_ball_x*cos(theta) + m_ball_y*sin(theta);
	double ytrans = m_ball_y*cos(theta) - m_ball_x*sin(theta);

        debug << "bool NUKick::chooseLeg()" << endl;
        debug << "theta - " << theta << endl;
        debug << "xtrans - " << xtrans << endl;
        debug << "ytrans - " << ytrans << endl;

        Vector2<float> ballLocation(m_ball_x, m_ball_y), leftFootRelativeBallLocation, rightFootRelativeBallLocation;

        Matrix leftFootTransform, rightFootTransform;
        m_data->getLeftLegTransform(leftFootTransform);
        m_data->getRightLegTransform(rightFootTransform);
        leftFootRelativeBallLocation = m_kinematicModel->TransformPositionToFoot(leftFootTransform, ballLocation);
        rightFootRelativeBallLocation = m_kinematicModel->TransformPositionToFoot(rightFootTransform, ballLocation);

        vector<float> leftPos = Kinematics::PositionFromTransform(leftFootTransform);
        vector<float> rightPos = Kinematics::PositionFromTransform(rightFootTransform);

        debug << "Right Pos = (" << rightPos[0] << "," << rightPos[1] << "," << rightPos[2] << ")" << endl;
        debug << "Left Pos = (" << leftPos[0] << "," << leftPos[1] << "," << leftPos[2] << ")" << endl;

        debug << "Ball Position - " << endl;
        debug << "Origin Relative: (" << ballLocation.x << "," << ballLocation.y << ")" << endl;
        debug << "Right Foot Relative: (" << rightFootRelativeBallLocation.x << "," << rightFootRelativeBallLocation.y << ")" << endl;
        debug << "Left Foot Relative: (" << leftFootRelativeBallLocation.x << "," << leftFootRelativeBallLocation.y << ")" << endl;

        const float fwdAngleRange = PI/4.0f;
        const float sideAngleRange = PI/8.0f;
        // Direction is forwardsish

        bool kickSelected = false;

        if(fabs(theta) < fwdAngleRange)
        {
            debug << "Right foot: " << endl << RightFootForwardKickableArea.MinX() << " < " << leftFootRelativeBallLocation.x << " < " << RightFootForwardKickableArea.MaxX() << endl;
            debug << RightFootForwardKickableArea.MinY() << " < " << leftFootRelativeBallLocation.y << " < " << RightFootForwardKickableArea.MaxY() << endl;
            debug << "Left foot: " << endl << LeftFootForwardKickableArea.MinX() << " < " << rightFootRelativeBallLocation.x << " < " << LeftFootForwardKickableArea.MaxX() << endl;
            debug << LeftFootForwardKickableArea.MinY() << " < " << rightFootRelativeBallLocation.y << " < " << LeftFootForwardKickableArea.MaxY() << endl;
            if(RightFootForwardKickableArea.PointInside(leftFootRelativeBallLocation.x,leftFootRelativeBallLocation.y))
            {
                if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                        return false;
                else if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                {
                    stop();
                    return false;
                }
                else
                {
                    m_kickingLeg = rightLeg;
                    m_swingDirection = ForwardSwing;
                    pose = PRE_KICK;
                    m_kickIsActive = true;
                    kickSelected = true;
                }
            }
            else if(LeftFootForwardKickableArea.PointInside(rightFootRelativeBallLocation.x,rightFootRelativeBallLocation.y))
            {

                if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                        return false;
                else if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                {
                    stop();
                    return false;
                }
                else
                {
                    m_kickingLeg = leftLeg;
                    m_swingDirection = ForwardSwing;
                    pose = PRE_KICK;
                    m_kickIsActive = true;
                    kickSelected = true;
                }
            }
        }
        // Direction is to the leftish
        else if(fabs(theta-PI/2.0f) < sideAngleRange)
        {
            if(RightFootLeftKickableArea.PointInside(m_ball_x,m_ball_y))
            {
                if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                        return false;
                else if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                {
                    stop();
                    return false;
                }
                else
                {
                    m_kickingLeg = rightLeg;
                    m_swingDirection = LeftSwing;
                    pose = PRE_KICK;
                    m_kickIsActive = true;
                    kickSelected = true;
                }
            }
            else if(LeftFootLeftKickableArea.PointInside(m_ball_x,m_ball_y))
            {
                if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                        return false;
                else if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                {
                    stop();
                    return false;
                }
                m_kickingLeg = leftLeg;
                m_swingDirection = LeftSwing;
                pose = PRE_KICK;
                m_kickIsActive = true;
                kickSelected = true;
            }
        }
        // Direction is rightish
        else if(fabs(theta+PI/2.0f) < sideAngleRange)
        {
            if(LeftFootRightKickableArea.PointInside(m_ball_x,m_ball_y))
            {
                if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                        return false;
                else if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                {
                    stop();
                    return false;
                }
                else
                {
                    m_kickingLeg = leftLeg;
                    m_swingDirection = RightSwing;
                    pose = PRE_KICK;
                    m_kickIsActive = true;
                    kickSelected = true;
                }
            }
            if(RightFootRightKickableArea.PointInside(m_ball_x,m_ball_y))
            {
                if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                        return false;
                else if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                {
                    stop();
                    return false;
                }
                else
                {
                    m_kickingLeg = rightLeg;
                    m_swingDirection = RightSwing;
                    pose = PRE_KICK;
                    m_kickIsActive = true;
                    kickSelected = true;
                }
            }
        }
        if(!kickSelected)
        {
            stop();
        }
        else
        {
            debug << "Kick Selected - Kicking Foot: " << toString(m_kickingLeg) << ", Swing Direction: " << toString(m_swingDirection) << endl;
        }
        return kickSelected;
}

double NUKick::MoveLimbToPositionWithSpeed(NUActionatorsData::bodypart_id_t limbId, vector<float> currentPosition, vector<float> targetPosition, float maxSpeed , float gain, float smoothness)
{
    const float movespeed = maxSpeed;
    const int numJoints = m_actions->getNumberOfJoints(limbId);
    vector< vector<float> > velocities;
    vector< vector<float> > positions;
    vector< vector<double> > times;

    // compute the time required to move into the initial pose for each limb
    double moveTime = 1000*(maxDifference(currentPosition, targetPosition)/movespeed);

    // set the move complettion to be the maximum of each limb
    //movecompletiontime = m_data->CurrentTime + moveTime;

    // give the command to the actionators
    //m_actions->addJointPositions(limbId, m_data->CurrentTime, currentPosition, velocity, gain);

    //MotionCurves::calculate(startTime,endTime, startAnkleAngle, endAnkleAngle, 1.0, 20.0f, ankleTimes, anklePositions, ankleVelocities);
    //m_actions->addJointPositions(a_kickingHipPitch,hipTimes,hipPositions,hipVelocities,100.0);
    float startTime = m_data->CurrentTime + 100.0;
    float endTime = startTime + moveTime;

    vector<double> endTimes(1,endTime);
    vector< vector<float> > endPositions(1,targetPosition);
    vector<float> gains(numJoints, gain);

    MotionCurves::calculate(startTime,endTimes,currentPosition,endPositions,smoothness,20.0f,times,positions,velocities);

    m_actions->addJointPositions(limbId,times,positions,velocities,gains);
    //m_actions->addJointPositions(limbId, m_data->CurrentTime + moveTime, targetPosition, velocity, gain);
    return endTime;
}
