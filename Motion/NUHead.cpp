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

#include <math.h>
#include <algorithm>
using namespace std;

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

    /*vector<float> sensorpositions;
    m_data->getJointPositions(NUSensorsData::HeadJoints, sensorpositions);
    
    vector<vector<double> > curvetimes;
    vector<vector<float> > curvepositions;
    vector<vector<float> > curvevelocities;
    MotionCurves::calculate(m_data->CurrentTime, times, sensorpositions, positions, 0.5, 10, curvetimes, curvepositions, curvevelocities);
    m_actions->addJointPositions(NUActionatorsData::HeadJoints, curvetimes, curvepositions, curvevelocities, m_default_gains);*/
    
    
    calculateBallPan();
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
    //!< @todo TODO: to calculate pans properly I need the ball size, camera offset, the camera field of view, and the field dimensions
    const float ballsize = 6.5;
    const float fieldwidth = 400;
    const float fieldlength = 600;
    
    const float cameraoffset = 0.6981;
    const float camerafov = 0.6012;
    
    // get the camera height, orientation and current head position from the sensor data
    float cameraheight;
    m_data->getCameraHeight(cameraheight);
    
    vector<float> orientation;
    m_data->getOrientation(orientation);
    float orientation_pitch = 0;
    if (orientation.size() > 2)
        orientation_pitch = orientation[1];
    
    float headpitch, headyaw;
    m_data->getJointPosition(NUSensorsData::HeadPitch, headpitch);
    m_data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
    
    // calculate the minimum and maximum scan lines
    float minpitch = std::min((float) (atan2(cameraheight, ballsize) - cameraoffset - 0.5*camerafov - orientation_pitch), m_pitch_limits[1]);
    float maxpitch = std::max((float) ((cameraheight - ballsize)/(1.1*sqrt(fieldwidth*fieldwidth + fieldlength*fieldlength)) - cameraoffset + 0.5*camerafov - orientation_pitch), m_pitch_limits[0]);
    
    // calculate scan lines required to scan the area between the min and max scan lines
    int numscans = (minpitch - maxpitch)/camerafov + 2;
    float spacing = (minpitch - maxpitch)/(numscans - 1);
    vector<float> scan_levels;
    if (fabs(headpitch - minpitch) < fabs(headpitch - maxpitch))
    {   // if we are closer to the minpitch position then start from the minpitch
        for (int i=0; i<numscans; i++)
            scan_levels.push_back(minpitch - i*spacing);
    }
    else
    {   // if we are closer to the maxpitch position then start from the maxpitch
        for (int i=0; i<numscans; i++)
            scan_levels.push_back(maxpitch + i*spacing);
    }
    for (unsigned int i=0; i<scan_levels.size(); i++)
        cout << scan_levels[i] << " " << (cameraheight - ballsize)/tan(scan_levels[i] + cameraoffset - 0.5*camerafov + orientation_pitch) << ", ";
    cout << endl;
    
    // now calculate the point sequence
    vector<vector<float> > scan_points;
    bool onleft = headyaw >= 0;
    vector<float> yawvalues;
    generateScan(scan_levels[0], headpitch, onleft, scan_points);
    for (unsigned int i=1; i<numscans; i++)
        generateScan(scan_levels[i], scan_levels[i-1], onleft, scan_points);
    
    for (unsigned int i=0; i<scan_points.size(); i++)
        cout << "[" << scan_points[i][0] << ", " << scan_points[i][1] << "], ";
    cout << endl;
    
    /* TEST HACK */
    vector<double> times(scan_points.size(), 0);
    for (unsigned int i=0; i<times.size(); i++)
        times[i] = i*1000 + m_data->CurrentTime;
    
    vector<float> sensorpositions;
    m_data->getJointPositions(NUSensorsData::HeadJoints, sensorpositions);
    
    vector<vector<double> > curvetimes;
    vector<vector<float> > curvepositions;
    vector<vector<float> > curvevelocities;
    MotionCurves::calculate(m_data->CurrentTime, times, sensorpositions, scan_points, 0.5, 10, curvetimes, curvepositions, curvevelocities);
    m_actions->addJointPositions(NUActionatorsData::HeadJoints, curvetimes, curvepositions, curvevelocities, m_default_gains);
    /* END TEST HACK */
}

void NUHead::generateScan(float pitch, float previouspitch, bool& onleft, vector<vector<float> >& scan)
{
    static vector<float> s(2,0);        // holds the special extra point when the yaw limits change
    static vector<float> a(2,0);        // holds the first of the scan line
    static vector<float> b(2,0);        // holds the second of the scan line
    
    cout << "pitch: " << pitch << " previouspitch: " << previouspitch << endl;
    
    int i = getPanLimitIndex(pitch);
    int p = getPanLimitIndex(previouspitch);
    if (i != p)
    {   // if the yaw limits change add the extra point to stop the head from hitting the shoulder
        cout << "pitches have different yaw limits: " << getPanLimitIndex(pitch) << " " << getPanLimitIndex(previouspitch) << endl;
        if (i < p)
            s[0] = m_pan_limits_pitch[i];
        else
            s[0] = m_pan_limits_pitch[p];
        if (onleft)
            s[1] = min(m_pan_limits_yaw[p][1], m_pan_limits_yaw[i][1]);
        else
            s[1] = max(m_pan_limits_yaw[p][0], m_pan_limits_yaw[i][0]);
        scan.push_back(s);
    }
    
    a[0] = pitch;
    b[0] = pitch;
    if (onleft)
    {
        a[1] = m_pan_limits_yaw[i][1];
        b[1] = m_pan_limits_yaw[i][0];
    }
    else
    {
        a[1] = m_pan_limits_yaw[i][0];
        b[1] = m_pan_limits_yaw[i][1];
    }
    scan.push_back(a);
    scan.push_back(b);
    onleft = !onleft;
}

/*! @brief Returns the index into the pan limit vectors for the given pitch value
    @param pitch the pitch value to get limit index for
    @return the index
 */
int NUHead::getPanLimitIndex(float pitch)
{
    for (unsigned int i=0; i<m_pan_limits_pitch.size(); i++)
    {
        if (pitch > m_pan_limits_pitch[i])
            return i;
    }
    return m_pan_limits_yaw.size()-1;
}

/*! @brief Returns true if the pan yaw limits change between pitch_a and pitch_b
    @param pitch_a a head pitch value in rad
    @param pitch_b a head pitch value in rad
    @return true if limits change false otherwise
 */
bool NUHead::panYawLimitsChange(float pitch_a, float pitch_b)
{
    int limit_a = getPanLimitIndex(pitch_a);
    int limit_b = getPanLimitIndex(pitch_b);
    
    if (limit_a != limit_b)
        return true;
    else
        return false;
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
    loadPanConfig();
}

/*! @brief Loads the maximum speed, maximum acceleration, and default gains from Head.cfg
 */
void NUHead::loadConfig()
{
    ifstream file((CONFIG_DIR + string("Motion/Head.cfg")).c_str());
    if (file.is_open() == false)
        errorlog << "NUHead::loadConfig(). Unable to open head configuration file" << endl;
    else
    {
        m_max_speeds = MotionFileTools::toFloatVector(file);
        m_max_accelerations = MotionFileTools::toFloatVector(file);
        m_default_gains = MotionFileTools::toFloatVector(file);
        m_pitch_limits = MotionFileTools::toFloatVector(file);
        m_yaw_limits = MotionFileTools::toFloatVector(file);
        file.close();
    }
}

/*! @brief Loads the pan configuration from HeadPan.cfg. 
 
 m_pan_limits_pitch and m_pan_limits_yaw are set such that they can be used to generate scan lines by:
 @code
 pitch = scan_level;
 if m_pan_limits_pitch[0] < scan_level:
    yaw = m_pan_limits_yaw[0]
 else if m_pan_limits[1] < scan_level:
    yaw = m_pan_limits_yaw[1]
 @endcode
 */
void NUHead::loadPanConfig()
{
    ifstream file((CONFIG_DIR + string("Motion/HeadPan.cfg")).c_str());
    if (file.is_open() == false)
        errorlog << "NUHead::loadPanConfig(). Unable to open head pan configuration file" << endl;
    else
    {
        float value;
        vector<float> range;
        while (!file.eof())
        {
            MotionFileTools::toFloatWithRange(file, value, range);
            if (range.size() == 2)
            {
                m_pan_limits_pitch.push_back(value);
                m_pan_limits_yaw.push_back(range);
            }
        }
        file.close();
    }
}



