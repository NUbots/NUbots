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
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Behaviour/Jobs/MotionJobs/KickJob.h"
#include "NUPlatform/NUSystem.h"

#ifdef USE_WALK
#include "Motion/NUWalk.h"
#endif // USE_WALK

#include "debug.h"
#include "debugverbositynumotion.h"
#include "Tools/Math/General.h"
using namespace mathGeneral;


NUKick::NUKick(NUWalk* walk)
{
    m_walk = walk;
    kinematicModel = new Kinematics();
    IKSys = new Legs();
    poseData.resize(4);
    pose = DO_NOTHING;
    m_kickingLeg = noLeg;

    m_kickIsActive = false;
    m_initialPositionSaved = false;
    m_leftStoredPosition.reserve(6);
    m_rightStoredPosition.reserve(6);

    m_currentTimestamp = 0;
    m_previousTimestamp = 0;
}

/*! @brief Destructor for motion module
 */
NUKick::~NUKick()
{
    kill();
    delete IKSys;
    delete kinematicModel;
}

/*! @brief Kills the kick module
 */
void NUKick::kill()
{
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
    doKick();
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

    m_ball_x = kickposition[0];
    m_ball_y = kickposition[1];

    m_target_x = kicktarget[0];
    m_target_y = kicktarget[1];
    m_target_timestamp = time;
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

	switch(pose)
	{
		case DO_NOTHING:
		{
			debug << "Choosing leg." << endl;
			chooseLeg();
			return;
		}

		case USE_LEFT_LEG:
		{	
			debug << "Left leg chosen. Lifting left leg." << endl;
			if(chooseLeg())
				return;
                        //liftLeg();
			return;
		}

		case USE_RIGHT_LEG:
		{
			debug << "Right leg chosen. Lifting right leg." << endl;
			if(chooseLeg())
				return;
			liftLeg();
			return;
		}

		case LIFT_LEG:
		{
			if(chooseLeg())
				return;
                        //setLeg();
			return;
		}
		/*
		case ADJUST_YAW: //none of this: yet!
		{
			if(chooseLeg())
				return;
			if(adjustYaw())
				return;
			setLeg();
			return;
		}
		*/
		case SET_LEG:
		{
			if(chooseLeg())
				return;
			//if(adjustYaw()) //not yet
			//	return;
			if(setLeg())
				return;
			poiseLeg();
			return;
		}
		
		case POISE_LEG:
		{
			if(chooseLeg())
				return;
			//if(adjustYaw()) //not yet
			//	return;
			if(setLeg())
				return;
			swing();
                        break;
		}
		
		case SWING:
		{
                        //retract();
                        break;
		}

		case RETRACT:
		{
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
	}
}

void NUKick::doKick()
{
    bool done = false;
    vector<float> pos (6, 0);
    vector<float> vel (6, 0);
    vector<float> gain (6, 100);

    vector<float> LeftLegTheta(6,0);
    vector<float> RightLegTheta(6,0);

    m_data->getJointPositions(NUSensorsData::LeftLegJoints, LeftLegTheta);
    m_data->getJointPositions(NUSensorsData::RightLegJoints, RightLegTheta);

	IKSys->inputLeft(LeftLegTheta);
	IKSys->inputRight(RightLegTheta);

        debug << "void NUKick::doKick()" << endl;
	switch(pose)
	{
		case DO_NOTHING:
		{
                        debug << "Doing Nothing." << endl;
			break;
		}
                case PRE_KICK:
                {
                    debug << "Doing Pre-Kick..." << endl;
                    done = doPreKick();
                    if(done)
                    {
                        cout << "Pre kick complete!" << endl;
                        debug << "Pre kick complete!" << endl;
                        pose = TRANSFER_TO_SUPPORT;
                    }
                    break;
                }
                case POST_KICK:
                {
                    debug << "Doing Post-Kick..." << endl;
                    done = doPostKick();
                    if(done)
                    {
                        cout << "Post kick complete!" << endl;
                        debug << "Post kick complete!" << endl;
                        m_kickIsActive = false;
                        pose = DO_NOTHING;
                    }
                    break;
                }
                case TRANSFER_TO_SUPPORT:
		{
                        debug << "Shifting Weight..." << endl;
                        legId_t supportLeg = noLeg;
                        if(m_kickingLeg == rightLeg)
                            supportLeg = leftLeg;
                        else if(m_kickingLeg == leftLeg)
                            supportLeg = rightLeg;
                        else
                        {
                            pose = POST_KICK;
                            break;
                        }
                        // Shift the weight of the robot to the support leg.
                        done = ShiftWeightToFoot(supportLeg,1.0f,0.04);
                        if(done)
                        {
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            cout << "Weight now on support foot!" << endl;
                            debug << "Weight now on support foot!" << endl;
                            pose = LIFT_LEG;
                        }
                        /*
			IKSys->useLeftLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                    */
			break;
		}

		case USE_RIGHT_LEG:
		{
                        debug << "Adding joint positions for right kick..." << endl;
			IKSys->useRightLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case LIFT_LEG:
		{
                        debug << "Lifting Leg..." << endl;
                        done = LiftKickingLeg(m_kickingLeg);
                        if(done)
                        {
                            m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            cout << "Leg is now lifted!" << endl;
                            debug << "Leg is now lifted!" << endl;
                            pose = SWING;
                        }
                        /*
			IKSys->liftLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                        */
			break;
		}

		case ADJUST_YAW:
		{
                        debug << "Adding joint positions to adjust yaw..." << endl;
			IKSys->adjustYaw(poseData[0]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			break;
		}

		case SET_LEG:
		{
                        debug << "Adding joint positions to set leg..." << endl;
			IKSys->setLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case POISE_LEG:
		{
                        debug << "Adding joint positions for poise leg..." << endl;
			IKSys->setLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case SWING:
		{
                        debug << "Swinging Leg..." << endl;
                        done = SwingLegForward(m_kickingLeg, 0.1);
                        if(done)
                        {
                            cout << "Swing completed!" << endl;
                            debug << "Swing completed!" << endl;
                            pose = RETRACT;
                        }
                        /*
                        debug << "Adding joint positions for swing..." << endl;
			IKSys->moveLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                        */
			break;
		}
                case RETRACT:
                    {
                        debug << "Retracting Leg..." << endl;
                        done = RetractLeg(m_kickingLeg);
                        if(done)
                        {
                            cout << "Leg Retracted!" << endl;
                            debug << "Leg Retracted!" << endl;
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            pose = REALIGN_LEGS;
                        }
                        break;
                    }
                case REALIGN_LEGS:
                    {
                        debug << "Lowering Leg..." << endl;
                        done = LowerLeg(m_kickingLeg);
                        if(done)
                        {
                            cout << "Legs Aligned!" << endl;
                            debug << "Legs Aligned!" << endl;
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            pose = UNSHIFT_LEG;
                        }
                        break;
                    }
                case UNSHIFT_LEG:
                {
                        debug << "Unshifting Weight..." << endl;
                        done = this->ShiftWeightToFoot(m_kickingLeg,0.5f,0.04);
                        if(done)
                        {
                            debug << "Weight Unshifted!" << endl;
                            //m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), LeftLegTheta, vel, gain);
                            //m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), RightLegTheta, vel, gain);
                            pose = POST_KICK;
                        }
                        break;
                }

                case RESET:
		{
                        debug << "Reset" << endl;
                        /*
                        debug << "Adding joint positions for reset..." << endl;
			IKSys->reset();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
                        */
			break;
		}

		case NO_KICK:
		{
                        debug << "No kick." << endl;
			break;
		}

		default:
		{	
                        debug << "unknown pose." << endl;
			pose = NO_KICK;
			break;
		}
	}
}

//bool NUKick::ShiftWeightToFoot(legId_t supportLeg, float targetWeightPercentage, float speed)
//{
//    const float shiftSpeed = speed;
//
//    bool validData = true;
//    bool leftContact, rightContact;
//    validData = validData && m_data->getFootContact(NUSensorsData::LeftFoot, leftContact);
//    validData = validData && m_data->getFootContact(NUSensorsData::RightFoot, rightContact);
//    if(!validData) return false;
//    float lcopx(0.0f),lcopy(0.0f),rcopx(0.0f),rcopy(0.0f);
//    float lforce(0.0f),rforce(0.0f);
//
//
//    float leftImpactTime, rightImpactTime;
//    bool recentLeftImpact = m_data->footImpact(NUSensorsData::LeftFoot,leftImpactTime);
//    bool recentRightImpact = m_data->footImpact(NUSensorsData::RightFoot,rightImpactTime);
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
//
//    vector<float>leftJoints;
//    vector<float>rightJoints;
//    validData = validData && m_data->getJointTargets(NUSensorsData::LeftLegJoints,leftJoints);
//    validData = validData && m_data->getJointTargets(NUSensorsData::RightLegJoints,rightJoints);
//    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);
//
//
//    if(validData)
//    {
//        vector<float> vel (6, 0);
//        vector<float> gain (6, 100);
//        float weightPercentage;
//        float deltaTheta;
//        bool recentImpact;
//        if(supportLeg == rightLeg)
//        {
//            weightPercentage = rforce / (rforce + lforce);
//            deltaTheta = shiftSpeed;
//            recentImpact = recentRightImpact;
//        }
//        else if(supportLeg == leftLeg)
//        {
//            weightPercentage = lforce / (rforce + lforce);
//            deltaTheta = -shiftSpeed;
//            recentImpact = recentLeftImpact;
//        }
//        debug << "weightPercentage = " << weightPercentage << endl;
//        if(weightPercentage < targetWeightPercentage)
//        {
//            debug << "Moving Joints..." << endl;
//            leftJoints[0] += deltaTheta;
//            leftJoints[4] -= deltaTheta;
//            rightJoints[0] += deltaTheta;
//            rightJoints[4] -= deltaTheta;
//        }
//        else if(!recentImpact)
//        {
//            return true;
//        }
//        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
//        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
//    }
//    return false;
//}

bool NUKick::doPreKick()
{
    bool validData = true;
    vector<float>leftJoints;
    vector<float>rightJoints;
    validData = validData && m_data->getJointPositions(NUSensorsData::LeftLegJoints,leftJoints);
    validData = validData && m_data->getJointPositions(NUSensorsData::RightLegJoints,rightJoints);
    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);

    if(!m_initialPositionSaved && validData)
    {
        m_initialPositionSaved = true;
        m_leftStoredPosition = leftJoints;
        m_rightStoredPosition = rightJoints;
    }

#ifdef USE_WALK
    if(m_walk)
    {
        vector<float> speed;
        m_walk->getCurrentSpeed(speed);

        if(speed.size() < 3) return false;

        if( (speed[0] != 0.0f) || (speed[1] != 0.0f) || (speed[2] != 0.0f))
        {
            return false;
        }
    }
#endif // USE_WALK

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, 100);
        vector<float>leftInitPose(6,0.0f);
        vector<float>rightInitPose(6,0.0f);

        float distanceFromTarget = 0.0f;
        float diff;
        for (int i = 0; i < leftInitPose.size(); i++)
        {
            diff = leftInitPose[i] - leftJoints[i];
            distanceFromTarget = sqrt(distanceFromTarget*distanceFromTarget + diff*diff);
            diff = rightInitPose[i] - rightJoints[i];
            distanceFromTarget = sqrt(distanceFromTarget*distanceFromTarget + diff*diff);
        }
        debug << "distanceFromTarget = " << distanceFromTarget << endl;
        if(distanceFromTarget < 0.01)
        {
            return true;
        }

        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftInitPose, vel, gain);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightInitPose, vel, gain);
    }
    return false;
}

bool NUKick::doPostKick()
{
    bool validData = true;
    vector<float>leftJoints;
    vector<float>rightJoints;
    validData = validData && m_data->getJointPositions(NUSensorsData::LeftLegJoints,leftJoints);
    validData = validData && m_data->getJointPositions(NUSensorsData::RightLegJoints,rightJoints);
    validData = validData && (leftJoints.size() >= 6) && (rightJoints.size() >= 6);

    if(!m_initialPositionSaved) return true;

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, 100);

        float distanceFromTarget = 0.0f;
        float diff;
        for (int i = 0; i < m_leftStoredPosition.size(); i++)
        {
            diff = m_leftStoredPosition[i] - leftJoints[i];
            distanceFromTarget = sqrt(distanceFromTarget*distanceFromTarget + diff*diff);
            diff = m_rightStoredPosition[i] - rightJoints[i];
            distanceFromTarget = sqrt(distanceFromTarget*distanceFromTarget + diff*diff);
        }
        debug << "distanceFromTarget = " << distanceFromTarget << endl;
        if(distanceFromTarget < 0.01)
        {
            m_initialPositionSaved = false;
            m_leftStoredPosition.clear();
            m_rightStoredPosition.clear();
            m_kickIsActive = false;
            return true;
        }

        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), m_leftStoredPosition, vel, gain);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), m_rightStoredPosition, vel, gain);
    }
}

bool NUKick::ShiftWeightToFoot(legId_t targetLeg, float targetWeightPercentage, float speed)
{
    const float maxShiftSpeed = speed;
    const float reqiredAccuracy = 0.05;
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
        vector<float> gain (6, 100);
        float weightPercentage;
        float weightError = 0;
        bool recentImpact;
        float newHipPos;
        if(targetLeg == rightLeg)
        {
            weightPercentage = rforce / (rforce + lforce);
            weightError = targetWeightPercentage - weightPercentage;
            recentImpact = recentRightImpact;
            recentImpact = m_data->CurrentTime - rightImpactTime < 33.0f;
            newHipPos = rightJoints[0] + crop(weightError*propGain, -maxShiftSpeed, maxShiftSpeed);
        }
        else if(targetLeg == leftLeg)
        {
            weightPercentage = lforce / (rforce + lforce);
            weightError = weightPercentage - targetWeightPercentage;
            recentImpact = recentLeftImpact;
            recentImpact = m_data->CurrentTime - leftImpactTime < 33.0f;
            newHipPos = leftJoints[0] + crop(weightError*propGain, -maxShiftSpeed, maxShiftSpeed);
        }
        if(fabs(weightError) > reqiredAccuracy)
        {

            leftJoints[0] = newHipPos;
            rightJoints[0] = newHipPos;
            leftJoints[4] = -newHipPos;
            rightJoints[4] = -newHipPos;
        }
        else if(!recentImpact)
        {
            return true;
        }
        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
    }
    return false;
}

bool NUKick::LiftKickingLeg(legId_t kickingLeg)
{
    float liftSpeed = 0.1;
    bool validData = true;

    NUSensorsData::foot_id_t s_supportFoot;
    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_supportLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(kickingLeg == rightLeg)
    {
        s_supportFoot = NUSensorsData::LeftFoot;
        s_kickingFoot = NUSensorsData::RightFoot;
        s_supportLeg = NUSensorsData::LeftLegJoints;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_supportLeg = NUActionatorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
    }
    else if(kickingLeg == leftLeg)
    {
        s_supportFoot = NUSensorsData::RightFoot;
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_supportLeg = NUSensorsData::RightLegJoints;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_supportLeg = NUActionatorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
    }
    else return true;

    bool supportContact, kickContact;
    validData = validData && m_data->getFootContact(s_supportFoot, supportContact);
    validData = validData && m_data->getFootContact(s_kickingFoot, kickContact);
    if(!validData) return false;
    float copx(0.0f),copy(0.0f);
    float force(0.0f);

    if(supportContact)
    {
        validData = validData && m_data->getFootCoP(s_supportFoot,copx,copy);
        validData = validData && m_data->getFootForce(s_supportFoot,force);
    }

    vector<float>supportLegJoints;
    vector<float>kickLegJoints;
    validData = validData && m_data->getJointTargets(s_supportLeg,supportLegJoints);
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickLegJoints);
    validData = validData && (supportLegJoints.size() >= 6) && (kickLegJoints.size() >= 6);

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, 100);

        // Balance using support leg
        BalanceCoP(supportLegJoints, copx, copy);
        // Keep kicking leg paralled in a lengthways orientation.
        kickLegJoints[0] = supportLegJoints[0];
        kickLegJoints[4] = supportLegJoints[4];

        if(kickLegJoints[3] < 2.0)
            kickLegJoints[1] -= liftSpeed;
        else
            kickLegJoints[1] += 0.5*liftSpeed;
        kickLegJoints[3] += 2*liftSpeed;
        kickLegJoints[5] -= liftSpeed;
        if(kickLegJoints[1] > 0.51) return true;

        m_actions->addJointPositions(a_supportLeg, nusystem->getTime(), supportLegJoints, vel, gain);
        m_actions->addJointPositions(a_kickingLeg, nusystem->getTime(), kickLegJoints, vel, gain);
    }
    return false;
}

bool NUKick::IsPastTime(float time){
    return (m_data->CurrentTime > time);
}

void NUKick::BalanceCoP(vector<float>& jointAngles, float CoPx, float CoPy)
{
    // Linear controller to centre CoP
    const float gainx = 0.001;
    const float gainy = 0.001;

    // Hip Roll
    jointAngles[0] += 0.5 * gainy * CoPy;
    // Ankle Roll
    jointAngles[4] = -jointAngles[0];

    jointAngles[5] += gainx * CoPx;
    return;
}

void NUKick::FlattenFoot(vector<float>& jointAngles)
{
    jointAngles[5] = -(jointAngles[1] + jointAngles[3]);
    return;
}

void NUKick::MaintainSwingHeight(legId_t supportLeg, vector<float>& supportLegJoints, legId_t swingLeg, vector<float>& swingLegJoints, float swingHeight)
{
    return;
}

bool NUKick::SwingLegForward(legId_t kickingLeg, float speed)
{
    float swingSpeed = speed;
    bool validData = true;

    NUSensorsData::foot_id_t s_supportFoot;
    NUSensorsData::foot_id_t s_kickingFoot;
    NUSensorsData::bodypart_id_t s_supportLeg;
    NUSensorsData::bodypart_id_t s_kickingLeg;
    NUActionatorsData::bodypart_id_t a_supportLeg;
    NUActionatorsData::bodypart_id_t a_kickingLeg;
    if(kickingLeg == rightLeg)
    {
        s_supportFoot = NUSensorsData::LeftFoot;
        s_kickingFoot = NUSensorsData::RightFoot;
        s_supportLeg = NUSensorsData::LeftLegJoints;
        s_kickingLeg = NUSensorsData::RightLegJoints;
        a_supportLeg = NUActionatorsData::LeftLegJoints;
        a_kickingLeg = NUActionatorsData::RightLegJoints;
    }
    else if(kickingLeg == leftLeg)
    {
        s_supportFoot = NUSensorsData::RightFoot;
        s_kickingFoot = NUSensorsData::LeftFoot;
        s_supportLeg = NUSensorsData::RightLegJoints;
        s_kickingLeg = NUSensorsData::LeftLegJoints;
        a_supportLeg = NUActionatorsData::RightLegJoints;
        a_kickingLeg = NUActionatorsData::LeftLegJoints;
    }
    else return true;


    bool supportFootContact;
    validData = validData && m_data->getFootContact(s_supportFoot, supportFootContact);

    if(!validData) return false;
    float copx(0.0f),copy(0.0f);
    float force(0.0f);

    if(supportFootContact)
    {
        validData = validData && m_data->getFootCoP(s_supportFoot,copx,copy);
        validData = validData && m_data->getFootForce(s_supportFoot,force);
    }

    vector<float>supportLegJoints;
    vector<float>kickingLegJoints;
    validData = validData && m_data->getJointTargets(s_supportLeg,supportLegJoints);
    validData = validData && m_data->getJointTargets(s_kickingLeg,kickingLegJoints);
    validData = validData && (supportLegJoints.size() >= 6) && (kickingLegJoints.size() >= 6);

    const float targetHipPitch = -1.2;
    const float targetKneePitch = 0.6f;

    if(validData)
    {
        vector<float> vel (6, 0);
        vector<float> gain (6, 100);

//        if(kickingLeg == rightLeg)
//        {
            float hipDiff = targetHipPitch - kickingLegJoints[1];
            float kneeDiff = targetKneePitch - kickingLegJoints[3];

            if((hipDiff > 0) && (kneeDiff > 0)) return true;

            float hipFramesRequired = fabs(hipDiff) / swingSpeed;
            float kneeFramesRequired = fabs(kneeDiff) / swingSpeed;

            float slowestJointSamplesLeft = 0.0f;
            if(hipFramesRequired > slowestJointSamplesLeft) slowestJointSamplesLeft = hipFramesRequired;
            if(kneeFramesRequired > slowestJointSamplesLeft) slowestJointSamplesLeft = kneeFramesRequired;

            debug << "swingSpeed = " << swingSpeed << endl;
            debug << "hipDiff = " << hipDiff << endl;
            debug << "kneeDiff = " << kneeDiff << endl;
            debug << "hipFramesRequired = " << hipFramesRequired << endl;
            debug << "kneeFramesRequired = " << kneeFramesRequired << endl;
            debug << "slowestJointSamplesLeft = " << slowestJointSamplesLeft << endl;

            BalanceCoP(supportLegJoints, copx, copy);
            kickingLegJoints[0] = supportLegJoints[0];
            kickingLegJoints[4] = supportLegJoints[4];

            if(slowestJointSamplesLeft - hipFramesRequired < 1.0)
            {
                kickingLegJoints[1] -= swingSpeed;
                debug << "moving Hip = " << kickingLegJoints[1] << endl;
            }
            if(slowestJointSamplesLeft - kneeFramesRequired < 1.0)
            {
                kickingLegJoints[3] -= swingSpeed;
                debug << "moving Knee = " << kickingLegJoints[3] << endl;
            }
            FlattenFoot(kickingLegJoints);

//        }
//        else if(kickingLeg == leftLeg)
//        {
//            float hipDiff = targetHipPitch - leftJoints[1];
//            float kneeDiff = targetKneePitch - leftJoints[3];
//
//            if((hipDiff > 0) && (kneeDiff > 0)) return true;
//
//            float hipFramesRequired = fabs(hipDiff) / swingSpeed;
//            float kneeFramesRequired = fabs(kneeDiff) / swingSpeed;
//
//            float slowestJointSamplesLeft = 0.0f;
//            if(hipFramesRequired > slowestJointSamplesLeft) slowestJointSamplesLeft = hipFramesRequired;
//            if(kneeFramesRequired > slowestJointSamplesLeft) slowestJointSamplesLeft = kneeFramesRequired;
//
//            debug << "swingSpeed = " << swingSpeed << endl;
//            debug << "hipDiff = " << hipDiff << endl;
//            debug << "kneeDiff = " << kneeDiff << endl;
//            debug << "hipFramesRequired = " << hipFramesRequired << endl;
//            debug << "kneeFramesRequired = " << kneeFramesRequired << endl;
//            debug << "slowestJointSamplesLeft = " << slowestJointSamplesLeft << endl;
//
//            BalanceCoP(rightJoints, rcopx, rcopy);
//            leftJoints[0] = rightJoints[0];
//            leftJoints[4] = rightJoints[4];
//
//            if(slowestJointSamplesLeft - hipFramesRequired < 1.0)
//            {
//                leftJoints[1] -= swingSpeed;
//                debug << "moving Hip = " << leftJoints[1] << endl;
//            }
//            if(slowestJointSamplesLeft - kneeFramesRequired < 1.0)
//            {
//                leftJoints[3] -= swingSpeed;
//                debug << "moving Knee = " << leftJoints[3] << endl;
//            }
//            FlattenFoot(leftJoints);
//        }

        m_actions->addJointPositions(a_supportLeg, nusystem->getTime(), supportLegJoints, vel, gain);
        m_actions->addJointPositions(a_kickingLeg, nusystem->getTime(), kickingLegJoints, vel, gain);
    }
    return false;
}

bool NUKick::RetractLeg(legId_t kickingLeg)
{
    const float maxSpeed = 0.05;
    bool validData = true;
    bool leftContact, rightContact;
    validData = validData && m_data->getFootContact(NUSensorsData::LeftFoot, leftContact);
    validData = validData && m_data->getFootContact(NUSensorsData::RightFoot, rightContact);
    if(!validData) return false;
    float lcopx(0.0f),lcopy(0.0f),rcopx(0.0f),rcopy(0.0f);
    float lforce(0.0f),rforce(0.0f);

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
        vector<float> gain (6, 100);

        const float targetHipPitch = -0.5;
        const float targetKneePitch = 1.0f;

        if(kickingLeg == rightLeg)
        {
            float hipDiff = targetHipPitch - rightJoints[1];
            float kneeDiff = targetKneePitch - rightJoints[3];
            if(sqrt(hipDiff*hipDiff + kneeDiff*kneeDiff) < 0.01) return true;

            BalanceCoP(leftJoints, lcopx, lcopy);
            rightJoints[0] = leftJoints[0];
            rightJoints[4] = leftJoints[4];

            rightJoints[1] += crop(hipDiff,-maxSpeed,maxSpeed);
            rightJoints[3] += crop(kneeDiff,-maxSpeed,maxSpeed);
            FlattenFoot(rightJoints);
        }
        else if(kickingLeg == leftLeg)
        {
            float hipDiff = targetHipPitch - leftJoints[1];
            float kneeDiff = targetKneePitch - leftJoints[3];
            if((fabs(hipDiff) + fabs(kneeDiff)) < 0.1) return true;

            BalanceCoP(rightJoints, lcopx, lcopy);
            leftJoints[0] = rightJoints[0];
            leftJoints[4] = rightJoints[4];

            leftJoints[1] += crop(hipDiff,-maxSpeed,maxSpeed);
            leftJoints[3] += crop(kneeDiff,-maxSpeed,maxSpeed);
            FlattenFoot(leftJoints);
        }
        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
    }
}

bool NUKick::LowerLeg(legId_t kickingLeg)
{
    const float maxSpeed = 0.1;
    const float pGain = 0.1;
    bool validData = true;
    bool leftContact, rightContact;
    validData = validData && m_data->getFootContact(NUSensorsData::LeftFoot, leftContact);
    validData = validData && m_data->getFootContact(NUSensorsData::RightFoot, rightContact);
    if(!validData) return false;
    float lcopx(0.0f),lcopy(0.0f),rcopx(0.0f),rcopy(0.0f);
    float lforce(0.0f),rforce(0.0f);

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
        vector<float> gain (6, 100);

        if(kickingLeg == rightLeg)
        {
            BalanceCoP(leftJoints, lcopx, lcopy);
            rightJoints[0] = leftJoints[0];
            rightJoints[4] = leftJoints[4];

            const float targetHipPitch = leftJoints[1];
            const float targetKneePitch = leftJoints[3];

            float hipDiff = targetHipPitch - rightJoints[1];
            float kneeDiff = targetKneePitch - rightJoints[3];
            if((fabs(hipDiff) + fabs(kneeDiff)) < 0.1) return true;

            rightJoints[1] += crop(pGain*hipDiff,-maxSpeed,maxSpeed);
            rightJoints[3] += crop(pGain*kneeDiff,-maxSpeed,maxSpeed);
            FlattenFoot(rightJoints);
        }
        else if(kickingLeg == leftLeg)
        {
            BalanceCoP(rightJoints, lcopx, lcopy);
            leftJoints[0] = rightJoints[0];
            leftJoints[4] = rightJoints[4];

            const float targetHipPitch = rightJoints[1];
            const float targetKneePitch = rightJoints[3];

            float hipDiff = targetHipPitch - leftJoints[1];
            float kneeDiff = targetKneePitch - leftJoints[3];
            if((fabs(hipDiff) + fabs(kneeDiff)) < 0.1) return true;

            leftJoints[1] += crop(pGain*hipDiff,-maxSpeed,maxSpeed);
            leftJoints[3] += crop(pGain*kneeDiff,-maxSpeed,maxSpeed);
            FlattenFoot(leftJoints);
        }
        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), leftJoints, vel, gain);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), rightJoints, vel, gain);
    }
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
        debug << "theta - " << theta << endl;
        debug << "xtrans - " << xtrans << endl;
        debug << "ytrans - " << ytrans << endl;

        if(!(xtrans>(3+FL)))
	{
		pose = NO_KICK;
                m_kickingLeg = noLeg;
                debug << "Unable to kick. Ball too close!" << endl;
		return true;
	}

	if(ytrans>(HZ+(3*theta/(PI/4))))
	{
            debug << "ball on right..." << endl;
            float KickabilityVariable = 10.0f;
            //float KickabilityVariable = (pow((ytrans-5-(3*theta/(PI/4))),2)+pow(xtrans,2));
            debug << "KickabilityVariable " << KickabilityVariable << endl;
            if(KickabilityVariable<36)
            {
                    //if(pose==USE_LEFT_LEG)
                    if(m_kickingLeg == rightLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
                            return false;
                    m_kickingLeg = rightLeg;
                    pose = PRE_KICK;
                    poseData[0] = 0;
                    poseData[0] = 0;
                    poseData[0] = 0;
                    poseData[0] = 500;
                    debug << "Right leg chosen." << endl;
                    m_kickIsActive = true;
                    return true;
            }
            else
            {
                    pose = NO_KICK;
                    m_kickingLeg = noLeg;
                    debug << "Unable to kick with right leg." << endl;
                    return true;
            }
	}
	else
	{
                debug << "ball on left..." << endl;
		if(ytrans<(-HZ-(3*theta/(PI/4))))
		{
                    float KickabilityVariable = 10.0f;
                    //float KickabilityVariable = (pow((ytrans-5-(3*theta/(PI/4))),2)+pow(xtrans,2));
                    debug << "KickabilityVariable " << KickabilityVariable << endl;
                        if(KickabilityVariable<36)
			{
                                //if(pose==USE_RIGHT_LEG)
                                if(m_kickingLeg == leftLeg && (pose != DO_NOTHING) && (pose != NO_KICK))
					return false;
                                m_kickingLeg = leftLeg;
                                pose = PRE_KICK;
				poseData[0] = 0;
				poseData[0] = 0;
				poseData[0] = 0;
				poseData[0] = 500;
                                debug << "Left leg chosen." << endl;
                                m_kickIsActive = true;
				return true;
			}
			else
			{
                                debug << "Unable to kick with left leg." << endl;
                                m_kickingLeg = noLeg;
				pose = NO_KICK;
				return true;
			}
		}
		else
		{
			pose = NO_KICK;
                        m_kickingLeg = noLeg;
                        debug << "Unable to kick with any leg." << endl;
			return true;
		}
	}
}

bool NUKick::liftLeg()
{
	
	if(IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2]<-2.5||IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2]>2.5)
	{
		return false;
	}
	else
	{
                //pose = LIFT_LEG;
		poseData[0] = 0.0;
		poseData[1] = 0.0;
		poseData[2] = 0.0;
		poseData[3] = 500;
	}

}

bool NUKick::adjustYaw()
{

	double theta = atan2(m_target_y-m_ball_y, m_target_x-m_ball_x);
	
	if(theta>0.0873)
	{
		pose = ADJUST_YAW;
		poseData[0] = theta;
		poseData[1] = 0.0;
		poseData[2] = 0.0;
		poseData[3] = 500;
		return true;
	}	
	return false;
}

bool NUKick::setLeg()
{
	double x;
	double y;
	double z;
	bool recal;
	if(IKSys->getLegInUse()==LEFT)
	{
		x = m_ball_x - FL - 3;
		y = m_ball_y;
		z = 3;

		if(x-IKSys->getLeftPosition()[0]<-0.5||x-IKSys->getLeftPosition()[0]>0.5)
		{
			recal = true;
		}
		if(y-IKSys->getLeftPosition()[1]<-0.5||y-IKSys->getLeftPosition()[1]>0.5)
		{
			recal = true;
		}
		if(z-(IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2])<-0.5||z-(IKSys->getLeftPosition()[2]-IKSys->getRightPosition()[2])>0.5)
		{
			recal = true;
		}
	}
	else
	{
		x = m_ball_x - FL - 3;
		y = m_ball_y;
		z = 3;

		if(x-IKSys->getRightPosition()[0]<-0.5||x-IKSys->getRightPosition()[0]>0.5)
		{
			recal = true;
		}
		if(y-IKSys->getRightPosition()[1]<-0.5||y-IKSys->getRightPosition()[1]>0.5)
		{
			recal = true;
		}
		if(z-(IKSys->getRightPosition()[2]-IKSys->getLeftPosition()[2])<-0.5||z-(IKSys->getRightPosition()[2]-IKSys->getLeftPosition()[2])>0.5)
		{
			recal = true;
		}
	}

	if(recal)
	{
		pose = SET_LEG;
		poseData[0] = x;
		poseData[1] = y;
		poseData[2] = z;
		poseData[3] = 500;
		
	}	
}

bool NUKick::poiseLeg()
{
	double x;
	double y;
	double z;
	
	pose = POISE_LEG;
	poseData[0] = m_ball_x - FL - 8;
	poseData[1] = m_ball_y;
	poseData[2] = 6.8;
	poseData[3] = 500;
	
	return false;
	
}

bool NUKick::swing()
{

	double x;
	double y;
	double z;
	
	pose = SWING;
	poseData[0] = 18;
	poseData[1] = 0;
	poseData[2] = 1.9;
	poseData[3] = 500;
	IKSys->setLeg(13, 10.6, 8.7);

	return false;

}

bool NUKick::retract()
{
	pose = RETRACT;
	return false;
}

