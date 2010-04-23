/*! @file NUHead.cpp
    @brief Implementation of nuhead class

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


#include "NUHead.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Head/PIDController.h"
#include "Tools/MotionCurves.h"
#include "debug.h"
#include "debugverbositynumotion.h"


NUHead::NUHead()
{
    m_pitch = 0;
    m_yaw = 0;
    
    m_pitch_pid = new PIDController(string("HeadPitch"), 0.6, 1.2/1000.0, 0.16*1000, -0.67, 0.51);
    m_yaw_pid = new PIDController(string("HeadYaw"), 0.6, 1.2/1000.0, 0.16*1000, -2.08, 2.08);
}

/*! @brief Destructor for motion module
 */
NUHead::~NUHead()
{
    if (m_pitch_pid != NULL) delete m_pitch_pid;
    if (m_yaw_pid != NULL) delete m_yaw_pid;
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUHead::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    doHead();
}


void NUHead::moveTo(const vector<double>& times, const vector<vector<float> >& positions)
{
    unsigned int length = times.size();
    if (m_data == NULL || length == 0 || positions.size() == 0)
        return;
    unsigned int width = positions[0].size();

    /*vector<float> vel (width, 0);
    vector<float> gain (width, 40);
    for (unsigned int i=0; i<length; i++)
    {
        m_actions->addJointPositions(NUActionatorsData::HeadJoints, times[i], positions[i], vel, gain);
    }*/
    vector<float> sensorpositions;
    m_data->getJointPositions(NUSensorsData::HeadJoints, sensorpositions);
    
    vector<vector<double> > curvetimes;
    vector<vector<float> > curvepositions;
    MotionCurves::calculate(m_data->CurrentTime, times, sensorpositions, positions, 0.5, 10, curvetimes, curvepositions);
    
    vector<vector<float> > curvevelocities;
    for (unsigned int i=0; i<curvetimes.size(); i++)
        curvevelocities.push_back(vector<float>(curvetimes[i].size(), 0));
    m_actions->addJointPositions(NUActionatorsData::HeadJoints, curvetimes, curvepositions, curvevelocities, 40);
    
    /*MotionCurves::calculate(m_data->CurrentTime, times[0], sensorpositions[0], positions[0][0], 0.5, 10, curvetimes, curvepositions);
    vector<float> curvevelocities(curvetimes.size(), 0);
    vector<float> curvegains(curvetimes.size(), 40);
    m_actions->addJointPositions(NUActionatorsData::HeadPitch, curvetimes, curvepositions, curvevelocities, curvegains);
    
    MotionCurves::calculate(m_data->CurrentTime, times[0], sensorpositions[1], positions[0][1], 0.5, 10, curvetimes, curvepositions);
    curvevelocities = vector<float>(curvetimes.size(), 0);
    curvegains = vector<float>(curvetimes.size(), 0);
    m_actions->addJointPositions(NUActionatorsData::HeadYaw, curvetimes, curvepositions, curvevelocities, curvegains);
    
    for (unsigned int i=1; i<times.size(); i++)
    {
        MotionCurves::calculate(times[i-1], times[i], positions[i-1][0], positions[i][0], 0.5, 10, curvetimes, curvepositions);
        vector<float> curvevelocities(curvetimes.size(), 0);
        vector<float> curvegains(curvetimes.size(), 40);
        m_actions->addJointPositions(NUActionatorsData::HeadPitch, curvetimes, curvepositions, curvevelocities, curvegains);
        
        MotionCurves::calculate(times[i-1], times[i], positions[i-1][1], positions[i][1], 0.5, 10, curvetimes, curvepositions);
        curvevelocities = vector<float>(curvetimes.size(), 0);
        curvegains = vector<float>(curvetimes.size(), 40);
        m_actions->addJointPositions(NUActionatorsData::HeadYaw, curvetimes, curvepositions, curvevelocities, curvegains);
    }*/
    
    /*cout << "MotionCurve: " << m_data->CurrentTime << " to " << times[0] << endl;
    for (unsigned int i=0; i<curvetimes.size(); i++)
        cout << curvetimes[i] << ", " << curvepositions[i] << endl;*/
}

void NUHead::doHead()
{
	/*static vector<float> pos (2,0);
	static vector<float> vel (2,0);
	static vector<float> gain (2,40);

    m_pitch_pid->setTarget(m_pitch);
    m_yaw_pid->setTarget(m_yaw);
    
    static vector<float> sensorpositions(2, 0);
    static vector<float> targetpositions(2, 0);
    m_data->getJointPositions(NUSensorsData::HeadJoints, sensorpositions);
    m_data->getJointTargets(NUSensorsData::HeadJoints, targetpositions);
    
    
	pos[0] = m_pitch;
	pos[1] = m_yaw;

	m_actions->addJointPositions(NUActionatorsData::HeadJoints, m_data->CurrentTime + 0, pos, vel, gain);*/
}

