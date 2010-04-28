/*! @file NUHead.h
    @brief Declaration of nuhead class
 
    @class NUHead
    @brief A module to provide head
 
    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2010 Jason Kulk, Jed Rietveld
 
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

#ifndef NUHEAD_H
#define NUHEAD_H

class NUSensorsData;
class NUActionatorsData;

class HeadJob;
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"

#include <vector>

class NUHead
{
public:
    NUHead();
    ~NUHead();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(HeadJob* job);
    void process(HeadPanJob* job);
    void process(HeadNodJob* job);
private:
    void moveTo(const std::vector<double>& times, const std::vector<std::vector<float> >& positions);
    void doHead();
    
    void getSensorValues();
    void calculateMinAndMaxPitch(float mindistance, float maxdistance, float& minpitch, float& maxpitch);
    void calculatePan();
    void calculateBallPan();
    void calculateBallAndLocalisationPan();
    void calculateLocalisationPan();
    
    vector<float> calculatePanLevels(float minpitch, float maxpitch);
    vector<vector<float> > calculatePanPoints(vector<float> levels);
    void generateScan(float pitch, float previouspitch, bool& onleft, vector<vector<float> >& scan);
    vector<double> calculatePanTimes(vector<vector<float> > points);
    int getPanLimitIndex(float pitch);
    bool panYawLimitsChange(float pitch_a, float pitch_b);
    
    void calculateNod();
    void calculateBallNod();
    void calculateBallAndLocalisationNod();
    void calculateLocalisationNod();
    
    void load();
    void loadConfig();
    void loadPanConfig();

private:
    NUSensorsData* m_data;                      //!< local pointer to the latest sensor data
    NUActionatorsData* m_actions;               //!< local pointer to the next actionators data
    
    float m_camera_height;                      //!< the camera height in cm
    float m_body_pitch;                         //!< the forward-backward lean of the robot is rad
    float m_sensor_pitch;                       //!< the sensor head pitch position
    float m_sensor_yaw;                         //!< the sensor head yaw position
    
    const float m_BALL_SIZE;
    const float m_FIELD_DIAGONAL;
    const float m_CAMERA_OFFSET;
    const float m_CAMERA_FOV_Y;
    
    bool m_is_panning;                          //!< true if we are currently panning the head
    HeadPanJob::head_pan_t m_pan_type;          //!< the type of pan we are currently performing
    vector<float> m_pan_limits_pitch;           //!< the corresponding pitch values for the yaw limits
    vector<vector<float> > m_pan_limits_yaw;    //!< the yaw limits of the pan (loaded from platform specific file)
    
    bool m_is_nodding;                          //!< true if we are currently nodding the head
    HeadNodJob::head_nod_t m_nod_type;          //!< the type of nod we are currently performing
    
    double m_move_end_time;                     //!< the time at which we need to resend the calculated curves to the actionators
    vector<vector<double> > m_curve_times;      //!< the motion curve times in ms
    vector<vector<float> > m_curve_positions;   //!< the motion curve positions in radians
    vector<vector<float> > m_curve_velocities;  //!< the motion curve velocities in radians
    
    vector<float> m_max_speeds;                 //!< the maximum speeds in rad/s
    vector<float> m_max_accelerations;          //!< the maximum accelerations in rad/s/s
    vector<float> m_default_gains;              //!< the default gains
    vector<float> m_pitch_limits;               //!< the pitch min and max
    vector<float> m_yaw_limits;                 //!< the yaw min and max
};

#endif

