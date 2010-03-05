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
#include "NUPlatform/NUSystem.h"
#include "debug.h"


NUKick::NUKick()
{
	IKSys = new Legs();
	poseData.resize(4);
	pose = DO_NOTHING;
}

/*! @brief Destructor for motion module
 */
NUKick::~NUKick()
{
    delete IKSys;
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
    doKick();
}


void NUKick::kickToPoint(const vector<float>& position, const vector<float>& target)
{
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
			liftLeg();
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
			setLeg();
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
		}
		
		case SWING:
		{
			retract();
		}

		case RETRACT:
		{
			
		}
	
		case RESET:
		{

		}

		case NO_KICK:
		{

		}
	}
}

void NUKick::doKick()
{
    static int frameCount = 0;

	vector<float> pos (6, 0);
    vector<float> vel (6, 0);
    vector<float> gain (6, 100);

	vector<float> LeftLegTheta(6,0);
	vector<float> RightLegTheta(6,0);

	m_data->getJointPositions(NUSensorsData::LeftLegJoints, LeftLegTheta);
	m_data->getJointPositions(NUSensorsData::RightLegJoints, RightLegTheta);

	IKSys->inputLeft(LeftLegTheta);
	IKSys->inputRight(RightLegTheta);

	switch(pose)
	{
		case DO_NOTHING:
		{
			break;
		}
		
		case USE_LEFT_LEG:
		{
			IKSys->useLeftLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case USE_RIGHT_LEG:
		{
			IKSys->useRightLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case LIFT_LEG:
		{
			IKSys->liftLeg();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case ADJUST_YAW:
		{
			IKSys->adjustYaw(poseData[0]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			break;
		}

		case SET_LEG:
		{
			IKSys->setLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case POISE_LEG:
		{
			IKSys->setLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case SWING:
		{
			IKSys->moveLeg(poseData[0], poseData[1], poseData[2]);
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
			break;
		}

		case RESET:
		{
			IKSys->reset();
			pos = IKSys->outputLeft();
			m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime(), pos, vel, gain);
			pos = IKSys->outputRight();
			m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime(), pos, vel, gain);
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

bool NUKick::chooseLeg()
{
	//currently must be at zero position
	double theta = atan2(m_target_y-m_ball_y, m_target_x-m_ball_x);
	
	//approximate, assume robot torso moves negligible amount and only rotates
	double xtrans = m_ball_x*cos(theta) + m_ball_y*sin(theta);
	double ytrans = m_ball_y*cos(theta) - m_ball_x*sin(theta);

	if(!(xtrans>3+FL))
	{
		pose = NO_KICK;
		return true;
	}

	if(ytrans>(HZ+(3*theta/(PI/4))))
	{
		if(pow((ytrans-5-(3*theta/(PI/4))),2)+pow(xtrans,2)<36)
		{
			if(pose==USE_LEFT_LEG)
				return false;
			pose = USE_LEFT_LEG;
			poseData[0] = 0;
			poseData[0] = 0;
			poseData[0] = 0;
			poseData[0] = 500;
			return true;
		}
		else
		{
			pose = NO_KICK;
			return true;
		}
	}
	else
	{
		if(ytrans<(-HZ-(3*theta/(PI/4))))
		{
			if(pow((ytrans+HZ+(3*theta/(PI/4))),2)+pow(xtrans,2)<36)
			{
				if(pose==USE_RIGHT_LEG)
					return false;
				pose = USE_RIGHT_LEG;
				poseData[0] = 0;
				poseData[0] = 0;
				poseData[0] = 0;
				poseData[0] = 500;
				return true;
			}
			else
			{
				pose = NO_KICK;
				return true;
			}
		}
		else
		{
			pose = NO_KICK;
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
		pose = LIFT_LEG;
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

