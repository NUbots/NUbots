/*! @file NUPlatform.h
    @brief Declaration of a abstract NUPlatform class, that defines the interface with the robotic platform.
 
    @class NUPlatform
    @brief An abstract class defining the interface to robotic platforms. The children of the NUPlatform
           implement the functionality for each of the different robots supported.

    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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

#ifndef NUPLATFORM_H
#define NUPLATFORM_H

class NUSensors;
class NUSensorsData;
class NUActionators;
class NUActionatorsData;
class NUCamera;

class JobList;
class NUIO;

#include "NUPlatform/NUCamera/CameraSettings.h"

#include <ctime>
#include <string>
#include <vector>

#ifdef __USE_POSIX199309                // Check if clock_gettime is avaliable
    #define __NU_SYSTEM_CLOCK_GETTIME 
    #define __NU_PERIODIC_CLOCK_NANOSLEEP
#else                                   // otherwise use boost.
    #include <boost/date_time/posix_time/posix_time.hpp>
    #include <boost/thread/thread.hpp>
#endif

class NUPlatform
{
public:
    enum LedIndices
    {
        Led0 = 0,
        Led1 = 1,
        Led2 = 2,
        Led3 = 3,
        Led4 = 4,
        Led5 = 5,
        Led6 = 6,
        Led7 = 7,
        NumLeds = 8
    };
public:
    NUPlatform();
    virtual ~NUPlatform();
 
    // Robot Identity functions
    std::string& getName();
    int getRobotNumber();
    int getTeamNumber();
    std::string& getMacAddress();
    
    // Time functions
    virtual long double getPosixTimeStamp();
    virtual double getTime();
    double getRealTime();
    double getProcessTime();
    double getThreadTime();
    
    // Sleep functions
    void msleep(double milliseconds);
    
    // Storage class access
    NUSensorsData* getNUSensorsData();
    NUActionatorsData* getNUActionatorsData();
    
    void updateImage();
    void updateSensors();
    void processActions();
    void process(JobList* jobs, NUIO* m_io);
    
    // Platform dependent functions
    virtual bool displayBatteryState();
    virtual bool verifySensors();
    virtual bool verifyVision(int framesdropped, int framesprocessed);
    virtual void add(const LedIndices& led, double time, const std::vector<float>& value);
    virtual void toggle(const LedIndices& led, double time, const std::vector<float>& value);
    
    void kill();
protected:
    void init();
    virtual void initName();
    virtual void initNumber();
    virtual void initTeam();
    virtual void initMAC();
private:
    void initClock();
    void initIdentity();

protected:
    NUCamera* m_camera;             //!< the robot's camera(s)
    NUSensors* m_sensors;           //!< the robot's sensors
    NUActionators* m_actionators;   //!< the robot's actionators
    
    std::string m_name;             //!< the robot's name
    int m_robot_number;             //!< the robot's number
    int m_team_number;              //!< the robot's team number
    std::string m_mac_address;      //!< the robot's MAC address (wired)
    
    int m_frames_zero_count;        //!< the number of consecutive times the frames processed has been zero
    int m_frames_dropped_count;     //!< the number of consecutive times the number of frames dropped is high
private:
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        struct timespec m_gettime_starttime;            //!< the program's start time according to gettime()
    #else
        boost::posix_time::ptime m_microsec_starttime;  //!< the program's start time according to boost::posix_time
    #endif
    long double m_time_offset;                          //!< an offset so that timesincestart = unixstamp - offset and unixstamp = timesincestart + offset
};

extern NUPlatform* Platform;                            //!< @relates NUPlatform a global pointer to the NUPlatform instance

#endif

