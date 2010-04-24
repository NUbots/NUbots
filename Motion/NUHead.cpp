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
#include "Tools/MotionCurves.h"
#include "Tools/MotionFileTools.h"

#include "Behaviour/Jobs/MotionJobs/HeadJob.h"

#include "debug.h"
#include "debugverbositynumotion.h"
#include "nubotdataconfig.h"


NUHead::NUHead()
{
    m_is_panning = false;
    m_is_nodding = false;
    m_next_add_time = 0;
    
    load();
}

/*! @brief Destructor for motion module
 */
NUHead::~NUHead()
{

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
}

/*! @brief Process a generic head job
    @param job the head job
 */
void NUHead::process(HeadJob* job)
{
    static vector<double> times;                // the times to reach each headposition tuple
    static vector<vector<float> > positions;    // a vector of headposition tuples
    
    m_is_panning = false;
    m_is_nodding = false;
    job->getPositions(times, positions);
    moveTo(times, positions);
}

/*! @brief Process a head pan job
    @param job the head pan job
 */
void NUHead::process(HeadPanJob* job)
{
    HeadPanJob::head_pan_t pantype = job->getPanType();
    if (m_is_panning == false || pantype != m_pan_type)
    {
        m_is_panning = true;
        m_is_nodding = false;
        m_pan_type = pantype;
        calculatePan();
    }
}

/*! @brief Process a head nod job
    @param job the nod job
 */
void NUHead::process(HeadNodJob* job)
{
    HeadNodJob::head_nod_t nodtype = job->getNodType();
    if (m_is_nodding == false || nodtype != m_nod_type)
    {
        m_is_nodding = true;
        m_is_panning = false;
        m_nod_type = nodtype;
        calculateNod();
    }
}


void NUHead::moveTo(const vector<double>& times, const vector<vector<float> >& positions)
{
    if (m_data == NULL || m_actions == NULL)
        return;

    vector<float> sensorpositions;
    m_data->getJointPositions(NUSensorsData::HeadJoints, sensorpositions);
    
    vector<vector<double> > curvetimes;
    vector<vector<float> > curvepositions;
    vector<vector<float> > curvevelocities;
    MotionCurves::calculate(m_data->CurrentTime, times, sensorpositions, positions, 0.5, 10, curvetimes, curvepositions, curvevelocities);
    m_actions->addJointPositions(NUActionatorsData::HeadJoints, curvetimes, curvepositions, curvevelocities, 40);
    
}

void NUHead::doHead()
{
	// at this stage there is nothing to do here
}

void NUHead::calculatePan()
{
    if (m_pan_type == HeadPanJob::Ball)
        calculateBallPan();
    else if (m_pan_type == HeadPanJob::BallAndLocalisation)
        calculateBallAndLocalisationPan();
    else if (m_pan_type == HeadPanJob::Localisation)
        calculateLocalisationPan();
}

void NUHead::calculateBallPan()
{
}

void NUHead::calculateBallAndLocalisationPan()
{
}

void NUHead::calculateLocalisationPan()
{
}

void NUHead::calculateNod()
{
    if (m_nod_type == HeadNodJob::Ball)
        calculateBallNod();
    else if (m_nod_type == HeadNodJob::BallAndLocalisation)
        calculateBallAndLocalisationNod();
    else if (m_nod_type == HeadNodJob::Localisation)
        calculateLocalisationNod();
}

void NUHead::calculateBallNod()
{
}

void NUHead::calculateBallAndLocalisationNod()
{
}

void NUHead::calculateLocalisationNod()
{
}

void NUHead::load()
{
    loadConfig();
}

void NUHead::loadConfig()
{
    ifstream file((CONFIG_DIR + string("Motion/Head.cfg")).c_str());
    if (file.is_open() == false)
    {
        errorlog << "NUHead::loadConfig(). Unable to open head configuration file" << endl;
    }
    else
    {
        m_max_speeds = MotionFileTools::toFloatVector(file);
        m_max_accelerations = MotionFileTools::toFloatVector(file);
        m_default_gains = MotionFileTools::toFloatVector(file);
        file.close();
    }
}



