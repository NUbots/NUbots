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

NUHead::NUHead() : m_BALL_SIZE(6.5), m_FIELD_DIAGONAL(721), m_CAMERA_OFFSET(0.6981), m_CAMERA_FOV_Y(0.6012)
{
    m_camera_height = 46;
    m_body_pitch = 0;
    m_sensor_pitch = 0;
    m_sensor_yaw = 0;
    
    m_is_panning = false;
    m_is_nodding = false;
    m_move_end_time = 0;
    
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
    
    if (m_move_end_time < m_data->CurrentTime)
    {
        if (m_is_panning)
            calculatePan();
        else if (m_is_nodding)
            calculateNod();
    }
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
    m_actions->addJointPositions(NUActionatorsData::HeadJoints, curvetimes, curvepositions, curvevelocities, m_default_gains);
}

/*! @brief Calculates the minimum and maximum head pitch values given a range on the field to look over
    @param mindistance the minimum distance in centimetres to look
    @param maxdistance the maximum distance in centimetres to look
    @param minpitch the calculated minpitch is stored here
    @param maxpitch the calculated maxpitch is stored here
 */
void NUHead::calculateMinAndMaxPitch(float mindistance, float maxdistance, float& minpitch, float& maxpitch)
{
    minpitch = std::min(static_cast<float>(atan2(m_camera_height, mindistance) - m_CAMERA_OFFSET - 0.5*m_CAMERA_FOV_Y - m_body_pitch), m_pitch_limits[1]);
    maxpitch = std::max(static_cast<float>(atan2(m_camera_height, maxdistance) - m_CAMERA_OFFSET + 0.5*m_CAMERA_FOV_Y - m_body_pitch), m_pitch_limits[0]);
}

void NUHead::calculatePan()
{
    getSensorValues();
    if (m_pan_type == HeadPanJob::Ball)
        calculateBallPan();
    else if (m_pan_type == HeadPanJob::BallAndLocalisation)
        calculateBallAndLocalisationPan();
    else if (m_pan_type == HeadPanJob::Localisation)
        calculateLocalisationPan();
}

void NUHead::calculateBallPan()
{
    calculateGenericPan(m_BALL_SIZE, 1.1*m_FIELD_DIAGONAL, m_pan_ball_speed);
}

void NUHead::calculateBallAndLocalisationPan()
{
    calculateGenericPan(m_BALL_SIZE, 1e10, min(m_pan_ball_speed, m_pan_localisation_speed));
}

void NUHead::calculateLocalisationPan()
{
    calculateGenericPan(120, 1e10, m_pan_localisation_speed);
}

void NUHead::calculateGenericPan(float mindistance, float maxdistance, float panspeed)
{
    float minpitch, maxpitch;
    calculateMinAndMaxPitch(mindistance, maxdistance, minpitch, maxpitch);
    
    vector<float> scan_levels = calculatePanLevels(minpitch, maxpitch);
    vector<vector<float> > scan_points = calculatePanPoints(scan_levels);
    vector<double> times = calculatePanTimes(scan_points, panspeed);
    
    
    for (unsigned int i=0; i<times.size(); i++)
    {
        cout << times[i] - times[0] << " " << MotionFileTools::fromVector(scan_points[i]) << endl;
    }
    moveTo(times, scan_points);
    
    if (times.size() > 0)
        m_move_end_time = times[times.size() -1];
    else
        m_move_end_time = m_data->CurrentTime;
}

/*! @brief Gets relevant sensor data from the NUSensorsData; sets m_camera_height, m_body_pitch, and m_sensor_pitch, m_sensor_yaw
 */
void NUHead::getSensorValues()
{
    // get the camera height, orientation and current head position from the sensor data
    float cameraheight;
    if (m_data->getCameraHeight(cameraheight))
        m_camera_height = cameraheight;
    else
        m_camera_height = 46;
    
    vector<float> orientation;
    m_data->getOrientation(orientation);
    if (orientation.size() > 2)
        m_body_pitch = orientation[1];
    
    m_data->getJointPosition(NUSensorsData::HeadPitch, m_sensor_pitch);
    m_data->getJointPosition(NUSensorsData::HeadYaw, m_sensor_yaw);
}

/*! @brief Calculates evenly spaced pitch values to pan at based on the camera field of view
 
    This function will automatically put the levels in the correct order based on the current head pitch
 
    @param minpitch the lowest pan level in radians
    @param maxpitch the hight pan level in radians
    @return a vector containing the ordered pan levels
 */
vector<float> NUHead::calculatePanLevels(float minpitch, float maxpitch)
{
    vector<float> levels;
    // calculate scan lines required to scan the area between the min and max scan lines
    int numscans = (minpitch - maxpitch)/m_CAMERA_FOV_Y + 2;
    float spacing = (minpitch - maxpitch)/(numscans - 1);
    
    if (fabs(m_sensor_pitch - minpitch) < fabs(m_sensor_pitch - maxpitch))
    {   // if we are closer to the minpitch position then start from the minpitch
        for (int i=0; i<numscans; i++)
            levels.push_back(minpitch - i*spacing);
    }
    else
    {   // if we are closer to the maxpitch position then start from the maxpitch
        for (int i=0; i<numscans; i++)
            levels.push_back(maxpitch + i*spacing);
    }
    return levels;
}

/*! @brief Calculates an ordered list of pan way points 
    @param levels the pan levels (in radians)
    @return a matrix containing [[pitch0, yaw0], [pitch1, yaw1], ...]
 */
vector<vector<float> > NUHead::calculatePanPoints(vector<float> levels)
{
    vector<vector<float> > points;
    bool onleft = m_sensor_yaw >= 0;
    
    if (levels.size() > 0)
        generateScan(levels[0], m_sensor_pitch, onleft, points);
    for (unsigned int i=1; i<levels.size(); i++)
        generateScan(levels[i], levels[i-1], onleft, points);

    return points;
}

/*! @brief Calculates a single scan to the given pitch value
 */
void NUHead::generateScan(float pitch, float previouspitch, bool& onleft, vector<vector<float> >& scan)
{
    static vector<float> s(2,0);        // holds the special extra point when the yaw limits change
    static vector<float> a(2,0);        // holds the first of the scan line
    static vector<float> b(2,0);        // holds the second of the scan line
    
    int i = getPanLimitIndex(pitch);
    int p = getPanLimitIndex(previouspitch);
    if (i != p)
    {   // if the yaw limits change add the extra point to stop the head from hitting the shoulder
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

/*! @brief Calculates the pan times based on the distance to the top of the scan line on the field
 */
vector<double> NUHead::calculatePanTimes(vector<vector<float> > points, float panspeed)
{
    vector<double> times;
    
    float distance, yawspeed, yawtime, pitchtime;
    if (points.size() > 0)
    {
        distance = m_camera_height/tan(points[0][0] + m_CAMERA_OFFSET - 0.5*m_CAMERA_FOV_Y + m_body_pitch);
        yawspeed = min(panspeed/distance, m_max_speeds[1]);
        yawtime = fabs(points[0][1] - m_sensor_yaw)/yawspeed;
        pitchtime = fabs(points[0][0] - m_sensor_pitch)/m_max_speeds[0];
        times.push_back(1000*max(yawtime, pitchtime) + m_data->CurrentTime);
    }
    for (unsigned int i=1; i<points.size(); i++)
    {
        if (i+1 < points.size()-1 and getPanLimitIndex(points[i][0]) != getPanLimitIndex(points[i+1][0]))       // hack to move at max speed when changing pan limits
            yawspeed = m_max_speeds[1];
        else
        {
            distance = m_camera_height/tan(points[i][0] + m_CAMERA_OFFSET - 0.5*m_CAMERA_FOV_Y + m_body_pitch);
            yawspeed = min(panspeed/distance, m_max_speeds[1]);
        }
        yawtime = fabs(points[i][1] - points[i-1][1])/yawspeed;
        pitchtime = fabs(points[i][0] - points[i-1][0])/m_max_speeds[0];
        times.push_back(1000*max(yawtime, pitchtime) + times[i-1]);
    }
    return times;
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
    {
        errorlog << "NUHead::loadConfig(). Unable to open head configuration file" << endl;
        m_max_speeds = vector<float>(2, 2);
        m_max_accelerations = vector<float>(2, 8);
        m_default_gains = vector<float>(2, 50);
        m_pitch_limits.push_back(-0.7); m_pitch_limits.push_back(0.7);
        m_yaw_limits.push_back(-1.57); m_yaw_limits.push_back(1.57);
    }
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
        m_pan_ball_speed = MotionFileTools::toFloat(file);
        m_pan_localisation_speed = MotionFileTools::toFloat(file);
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



