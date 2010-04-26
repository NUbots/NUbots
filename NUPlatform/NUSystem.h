/*! @file NUSystem.h
    @brief Declaration of a base nusystem class

    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef NUSYSTEM_H
#define NUSYSTEM_H

#include <time.h>
#ifdef __USE_POSIX199309                // Check if clock_gettime is avaliable
    #define __NU_SYSTEM_CLOCK_GETTIME 
#else                                   // otherwise use boost.
    #include <boost/date_time/posix_time/posix_time.hpp>
    using namespace boost::posix_time;
#endif

class NUSensorsData;
class NUActionatorsData;

class NUSystem
{
public:
    NUSystem();
    virtual ~NUSystem();
    // time functions
    virtual long double getPosixTimeStamp();
    virtual double getTime();
    virtual double getTimeFast(); 
    static long double getTimeOffset();
    
    static double getRealTime();       
    static double getRealTimeFast();
    static double getProcessTime();    
    static double getThreadTime();
    // battery functions
    virtual void displayBatteryState(NUSensorsData* data, NUActionatorsData* actions);
    // watchdog functions
    virtual void displayRunning(NUActionatorsData* actions);
    virtual void displayVisionFrameDrop(NUActionatorsData* actions);
private:
    // System time members
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        static struct timespec m_gettime_starttime;            //!< the program's start time according to gettime()
        static struct timespec m_gettimefast_starttime;        //!< the program's start time according to the fast verion of gettime()
    #else
        static ptime m_microsec_starttime;                     //!< the program's start time according to boost::posix_time
    #endif
    static long double m_time_offset;                          //!< an offset so that timesincestart = unixstamp - offset and unixstamp = timesincestart + offset
};

extern NUSystem* nusystem;           //!< This is an omnipresent variable for the underlying system. Having a global variable is a compromise, hopefully it will be only exception @relates NUSystem

#endif

