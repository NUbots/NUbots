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
class HeadTrackJob;
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"

#include <vector>

class NUHead
{
public:
    NUHead();
    ~NUHead();
    void kill();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(HeadJob* job);
    void process(HeadTrackJob* job);
    void process(HeadPanJob* job);
    void process(HeadNodJob* job);
private:
    void moveTo(const std::vector<double>& times, const std::vector<std::vector<float> >& positions);
    void doHead();
    
    void calculateHeadTarget(float elevation, float bearing, float centreelevation, float centrebearing, vector<double>& times, vector<vector<float> >& positions);
    
    void calculatePan();
    void calculateBallPan();
    void calculateBallAndLocalisationPan();
    void calculateLocalisationPan();
    void calculateGenericPan(float mindistance, float maxdistance, float panspeed);
    
    void getSensorValues();
    void calculateMinAndMaxPitch(float mindistance, float maxdistance, float& minpitch, float& maxpitch);
    vector<float> calculatePanLevels(float minpitch, float maxpitch);
    vector<vector<float> > calculatePanPoints(const vector<float>& levels);
    void generateScan(float pitch, float previouspitch, bool& onleft, vector<vector<float> >& scan);
    vector<double> calculatePanTimes(const vector<vector<float> >& points, float panspeed);
    int getPanLimitIndex(float pitch);
    bool panYawLimitsChange(float pitch_a, float pitch_b);
    
    void calculateNod();
    void calculateBallNod();
    void calculateBallAndLocalisationNod();
    void calculateLocalisationNod();
    void calculateGenericNod(float mindistance, float maxdistance, float nodspeed);
    vector<vector<float> > calculateNodPoints(float minpitch, float maxpitch);
    vector<double> calculateNodTimes(const vector<vector<float> >& points, float nodspeed);
    
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
    const float m_CAMERA_FOV_X;
    const float m_CAMERA_FOV_Y;
    
    bool m_is_panning;                          //!< true if we are currently panning the head
    HeadPanJob::head_pan_t m_pan_type;          //!< the type of pan we are currently performing
    float m_pan_ball_speed;                     //!< the speed of pans looking for the ball (Loaded from HeadPan.cfg)
    float m_pan_localisation_speed;             //!< the speed of pans looking for field objects that aren't the ball (Loaded from HeadPan.cfg)
    vector<float> m_pan_limits_pitch;           //!< the corresponding pitch values for the yaw limits (Loaded from HeadPan.cfg)
    vector<vector<float> > m_pan_limits_yaw;    //!< the yaw limits of the pan (Loaded from HeadPan.cfg)
    
    bool m_is_nodding;                          //!< true if we are currently nodding the head
    HeadNodJob::head_nod_t m_nod_type;          //!< the type of nod we are currently performing
    float m_nod_centre;                         //!< the centre yaw angle for the nod
    
    double m_move_end_time;                     //!< the time at which we need to resend the calculated curves to the actionators
    vector<vector<double> > m_curve_times;      //!< the motion curve times in ms
    vector<vector<float> > m_curve_positions;   //!< the motion curve positions in radians
    vector<vector<float> > m_curve_velocities;  //!< the motion curve velocities in radians
    
    vector<float> m_max_speeds;                 //!< the maximum speeds in rad/s (Loaded from Head.cfg. It is very important that head can move at these maximum speeds!
    vector<float> m_max_accelerations;          //!< the maximum accelerations in rad/s/s (Loaded from Head.cfg)
    vector<float> m_default_gains;              //!< the default gains (Loaded from Head.cfg)
    vector<float> m_pitch_limits;               //!< the pitch min and max (Loaded from Head.cfg)
    vector<float> m_yaw_limits;                 //!< the yaw min and max (Loaded from Head.cfg)
};

#endif

