/*! @file NUHead.cpp
    @brief Implementation of nuhead class

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"

#include "debug.h"
#include "debugverbositynumotion.h"
#include "nubotdataconfig.h"

#include <math.h>
#include <algorithm>
using namespace std;

NUHead::NUHead() : m_BALL_SIZE(6.5), m_FIELD_DIAGONAL(721), m_CAMERA_OFFSET(0.6981), m_CAMERA_FOV_X(0.8098), m_CAMERA_FOV_Y(0.6074)
{
    m_data = NULL;
    m_actions = NULL;
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
    kill();
}

/*! @brief Kills the head module
 */
void NUHead::kill()
{
    m_is_nodding = false;
    m_is_panning = false;
    if (m_data and m_actions)
    {
        m_move_end_time = m_data->CurrentTime;
        m_actions->addJointPositions(NUActionatorsData::HeadJoints, 0, vector<float>(2,0), vector<float>(2,0), 0);
    }
}

/*! @brief Returns the completion time of the current head movement.
 */
double NUHead::getCompletionTime()
{
    return m_move_end_time;
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
    vector<double> times;                // the times to reach each headposition tuple
    vector<vector<float> > positions;    // a vector of headposition tuples
    
    m_is_panning = false;
    m_is_nodding = false;
    job->getPositions(times, positions);
    moveTo(times, positions);
}

/*! @brief Process a generic head job
 @param job the head job
 */
void NUHead::process(HeadTrackJob* job)
{
    vector<double> times;                // the times to reach each headposition tuple
    vector<vector<float> > positions;    // a vector of headposition tuples
    
    m_is_panning = false;
    m_is_nodding = false;
    
    float elevation, bearing, centreelevation, centrebearing;
    job->getData(elevation, bearing, centreelevation, centrebearing);    
    calculateHeadTarget(elevation, bearing, centreelevation, centrebearing, times, positions);

    moveTo(times, positions);
}

/*! @brief Process a head pan job
    @param job the head pan job
 */
void NUHead::process(HeadPanJob* job)
{
    HeadPanJob::head_pan_t pantype = job->getPanType();
    m_pan_default_values = job->useDefaultValues();
    if (not m_pan_default_values)
    {
        job->getX(m_x_min, m_x_max);
        job->getYaw(m_yaw_min, m_yaw_max);
    }
    
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
        m_nod_centre = job->getCentreAngle();
        calculateNod();
    }
}

/*! @brief Calculates a motion curve for the given times and positions, then gives the curves to the actionators
    @param times the times in ms for each point on the motion sequence
    @param positions the motion sequence [[roll,pitch,yaw], [roll,pitch,yaw], ...[roll,pitch,yaw]]
 */
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

/*! @brief Calculates the head target from the image elevation and image bearing
    @param elevation the image elevation
    @param bearing the image bearing
    @param centreelevation the desired elevation in the image
    @param centrebearing the desired bearing in the image
    @param times will be updated
    @param positions will be updated
 */
void NUHead::calculateHeadTarget(float elevation, float bearing, float centreelevation, float centrebearing, vector<double>& times, vector<vector<float> >& positions)
{
    if (m_data and m_actions)
    {
        getSensorValues();
        const float gain_pitch = 0.8;           // proportional gain in the pitch direction
        const float gain_yaw = 0.6;             // proportional gain in the yaw direction
        
        float c_pitch = -centreelevation;
        float c_yaw = -centrebearing;
        
        float e_pitch = c_pitch - elevation;
        float e_yaw = c_yaw - bearing;
        
        float new_pitch = m_sensor_pitch - gain_pitch*e_pitch;
        float new_yaw = m_sensor_yaw - gain_yaw*e_yaw;
        
        times.push_back(m_data->CurrentTime);
        
        // clip the head targets to 'limits'
        float min_pitch = (m_CAMERA_FOV_Y/2 - m_CAMERA_OFFSET - m_body_pitch);
        float max_pitch = m_pitch_limits[1];
        if (new_pitch < min_pitch)
            new_pitch = min_pitch;
        else if (new_pitch > max_pitch)
            new_pitch = max_pitch;
        
        vector<float> target(2,0);
        target[0] = new_pitch;
        target[1] = new_yaw;
        positions.push_back(target);
    }
}

/*! @brief Calculates the minimum and maximum head pitch values given a range on the field to look over
    @param mindistance the minimum distance in centimetres to look
    @param maxdistance the maximum distance in centimetres to look
    @param minpitch the calculated minpitch is stored here
    @param maxpitch the calculated maxpitch is stored here
 */
void NUHead::calculateMinAndMaxPitch(float mindistance, float maxdistance, float& minpitch, float& maxpitch)
{
    float maxtilt_limit = m_CAMERA_FOV_Y/2 - m_CAMERA_OFFSET - m_body_pitch;
    minpitch = std::min(static_cast<float>(atan2(m_camera_height, mindistance) - m_CAMERA_OFFSET - 0.5*m_CAMERA_FOV_Y - m_body_pitch), m_pitch_limits[1]);
    maxpitch = std::max(static_cast<float>(atan2(m_camera_height, maxdistance) - m_CAMERA_OFFSET + 0.5*m_CAMERA_FOV_Y - m_body_pitch), maxtilt_limit);
    
    if (minpitch <= maxpitch)
    {
        float pitch = atan2(m_camera_height, (mindistance + maxdistance)/2) - m_CAMERA_OFFSET - m_body_pitch;
        if (pitch > m_pitch_limits[1])
            pitch = m_pitch_limits[1];
        else if (pitch < maxtilt_limit)
            pitch = maxtilt_limit;
        minpitch = pitch;
        maxpitch = pitch;
    }
}

/*! @brief Calculates the a new motion sequence for the current pan type, and sends it to the actionators
 */
void NUHead::calculatePan()
{
    if (m_data and m_actions)
    {
        getSensorValues();
        if (m_pan_type == HeadPanJob::Ball)
            calculateBallPan();
        else if (m_pan_type == HeadPanJob::BallAndLocalisation)
            calculateBallAndLocalisationPan();
        else if (m_pan_type == HeadPanJob::Localisation)
            calculateLocalisationPan();
    }
}

void NUHead::calculateBallPan()
{
    if (m_pan_default_values)
        calculateGenericPan(m_BALL_SIZE, 1.1*m_FIELD_DIAGONAL, m_yaw_limits[0], m_yaw_limits[1], m_pan_ball_speed);
    else
        calculateGenericPan(m_x_min, m_x_max, m_yaw_min, m_yaw_max, m_pan_ball_speed);
}

void NUHead::calculateBallAndLocalisationPan()
{
    if (m_pan_default_values)
        calculateGenericPan(m_BALL_SIZE, 1e10, m_yaw_limits[0], m_yaw_limits[1], min(m_pan_ball_speed, m_pan_localisation_speed));
    else
        calculateGenericPan(m_x_min, m_x_max, m_yaw_min, m_yaw_max, min(m_pan_ball_speed, m_pan_localisation_speed));
}

void NUHead::calculateLocalisationPan()
{
    if (m_pan_default_values)
        calculateGenericPan(120, 1e10, m_yaw_limits[0], m_yaw_limits[1], m_pan_localisation_speed);
    else
        calculateGenericPan(m_x_min, m_x_max, m_yaw_min, m_yaw_max, m_pan_localisation_speed);
}

/*! @brief Calculates a pan between mindistance (cm) and maxdistance (cm) at panspeed (cm/s)
 */
void NUHead::calculateGenericPan(float mindistance, float maxdistance, float minyaw, float maxyaw, float panspeed)
{
    float minpitch, maxpitch;
    calculateMinAndMaxPitch(mindistance, maxdistance, minpitch, maxpitch);
    
    vector<float> scan_levels = calculatePanLevels(minpitch, maxpitch);
    vector<vector<float> > scan_points = calculatePanPoints(scan_levels, minyaw, maxyaw);
    vector<double> times = calculatePanTimes(scan_points, panspeed);
    
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
    int numscans;       // the number of scans (pans)
    float spacing;      // the pitch spacing between each scan (radians)
    if (minpitch == maxpitch)
    {   // special case: only a single pan is required when the minpitch equal to maxpitch
        numscans = 1;
        spacing = 0;
    }
    else
    {
        numscans = int((minpitch - maxpitch)/m_CAMERA_FOV_Y + 2);
        spacing = (minpitch - maxpitch)/(numscans - 1);
    }
    
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
vector<vector<float> > NUHead::calculatePanPoints(const vector<float>& levels, float minyaw, float maxyaw)
{
    vector<vector<float> > points;
    bool onleft = m_sensor_yaw >= 0;
    
    if (levels.size() > 0)
        generateScan(levels[0], m_sensor_pitch, minyaw, maxyaw, onleft, points);
    for (unsigned int i=1; i<levels.size(); i++)
        generateScan(levels[i], levels[i-1], minyaw, maxyaw, onleft, points);

    return points;
}

/*! @brief Calculates a single scan to the given pitch value
 */
void NUHead::generateScan(float pitch, float previouspitch, float minyaw, float maxyaw, bool& onleft, vector<vector<float> >& scan)
{
    // Note. This function is confusing ;(.
    //      - m_pan_limits_yaw[i][0] is actually the right limit and m_pan_limits_yaw[i][1] is the left
    //        This means we need to match minyaw with m_pan_limits_yaw[i][0] and maxyaw with m_pan_limits_yaw[i][1]
    //      - onleft == true means we are currently on the left
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
        
        // now that we have made the point see if we actually need it by comparing it to the min and max yaw
        if ((onleft and maxyaw > s[1]) or (not onleft and minyaw < s[1]))
            scan.push_back(s);
    }
    
    a[0] = pitch;
    b[0] = pitch;
    if (onleft)
    {
        a[1] = min(m_pan_limits_yaw[i][1], maxyaw);         // pick the smallest of the limit and the desired
        b[1] = max(m_pan_limits_yaw[i][0], minyaw);         // pick the largest
    }
    else
    {
        a[1] = max(m_pan_limits_yaw[i][0], minyaw);
        b[1] = min(m_pan_limits_yaw[i][1], maxyaw);
    }
    scan.push_back(a);
    scan.push_back(b);
    onleft = !onleft;
}

/*! @brief Calculates the pan times based on the distance to the top of the scan line on the field
 */
vector<double> NUHead::calculatePanTimes(const vector<vector<float> >& points, float panspeed)
{
    vector<double> times;
    
    float distance, yawspeed, yawtime, pitchtime;
    if (points.size() > 0)
    {
        float ratio_hl = tan(points[0][0] + m_CAMERA_OFFSET - 0.5*m_CAMERA_FOV_Y + m_body_pitch);
        if (ratio_hl < 0.05)            // need to be careful here to avoid divide by zero, and VERY slow pan when the distance is close to infinity
            distance = 1.1*m_FIELD_DIAGONAL;
        else
            distance = m_camera_height/ratio_hl;
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
            float ratio_hl = tan(points[i][0] + m_CAMERA_OFFSET - 0.5*m_CAMERA_FOV_Y + m_body_pitch);
            if (ratio_hl < 0.05)        // need to be careful here to avoid divide by zero, and VERY slow pan when the distance is close to infinity
                distance = 1.1*m_FIELD_DIAGONAL;
            else
                distance = m_camera_height/ratio_hl;
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

/*! @brief Calculates a new motion curve for the currently selected nod type
 */
void NUHead::calculateNod()
{
    if (m_data and m_actions)
    {
        getSensorValues();
        if (m_nod_type == HeadNodJob::Ball)
            calculateBallNod();
        else if (m_nod_type == HeadNodJob::BallAndLocalisation)
            calculateBallAndLocalisationNod();
        else if (m_nod_type == HeadNodJob::Localisation)
            calculateLocalisationNod();
    }
}

void NUHead::calculateBallNod()
{
    calculateGenericNod(m_BALL_SIZE, 1.1*m_FIELD_DIAGONAL, m_pan_ball_speed);
}

void NUHead::calculateBallAndLocalisationNod()
{
    calculateGenericNod(m_BALL_SIZE, 1e10, std::min(m_pan_ball_speed, m_pan_localisation_speed));
}

void NUHead::calculateLocalisationNod()
{
    calculateGenericNod(120, 1e10, m_pan_localisation_speed);
}

/*! @brief Calculates a nod between mindistance and maxdistance (cm) at nodspeed (cm/s). 
           In fact, m_nod_centre is used to control the speed in most cases.
 */
void NUHead::calculateGenericNod(float mindistance, float maxdistance, float nodspeed)
{
    float minpitch, maxpitch;
    calculateMinAndMaxPitch(mindistance, maxdistance, minpitch, maxpitch);
    vector<vector<float> > points = calculateNodPoints(minpitch, maxpitch);
    vector<double> times = calculateNodTimes(points, nodspeed);
    
    moveTo(times, points);
    
    if (times.size() > 0)
        m_move_end_time = times.back();
    else
        m_move_end_time = m_data->CurrentTime;
}

/*! @brief Orders the min and max pitch values */
vector<vector<float> > NUHead::calculateNodPoints(float minpitch, float maxpitch)
{
    vector<vector<float> > points;
    points.reserve(2);
    vector<float> point(2,m_nod_centre);
    if (fabs(m_sensor_pitch - minpitch) < fabs(m_sensor_pitch - maxpitch))
    {   // if we are closer to the minpitch position then start from the minpitch
        point[0] = minpitch;
        points.push_back(point);
        point[0] = maxpitch;
        points.push_back(point);
    }
    else
    {
        point[0] = maxpitch;
        points.push_back(point);
        point[0] = minpitch;
        points.push_back(point);
    }
    return points;
}

/*! @brief Calculates the times for each point in the nod */
vector<double> NUHead::calculateNodTimes(const vector<vector<float> >& points, float nodspeed)
{
    vector<double> times;
    
    if (points.size() >= 2)
    {
        // we use the usual formula to determine the speed to the first point
        float ratio_hl = tan(points[0][0] + m_CAMERA_OFFSET - 0.5*m_CAMERA_FOV_Y + m_body_pitch);
        float distance = 1.1*m_FIELD_DIAGONAL;
        if (ratio_hl > 0.05)            // need to be careful here to avoid divide by zero, and VERY slow pan when the distance is close to infinity
            distance = m_camera_height/ratio_hl;
        
        float yawspeed = min(nodspeed/distance, m_max_speeds[1]);
        float yawtime = fabs(points[0][1] - m_sensor_yaw)/yawspeed;
        float pitchspeed = std::min(nodspeed/distance, m_max_speeds[0]);                       // the same pitch speed is used for both points
        float pitchtime = fabs(points[0][0] - m_sensor_pitch)/pitchspeed;
        
        times.push_back(1000*max(yawtime, pitchtime) + m_data->CurrentTime); 
        
        // however, for the second point we assume that m_nod_centre is also the yaw speed of the robot itself
        float nodtime = 10;
        if (fabs(m_nod_centre) > 0.05)
            nodtime = (m_CAMERA_FOV_X/3.0)/fabs(m_nod_centre);
        
        times.push_back(1000*nodtime + times.back());
    }
    return times;
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



