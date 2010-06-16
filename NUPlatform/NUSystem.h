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
#include <string>
#ifdef __USE_POSIX199309                // Check if clock_gettime is avaliable
    #define __NU_SYSTEM_CLOCK_GETTIME 
    #define __NU_PERIODIC_CLOCK_NANOSLEEP
#else                                   // otherwise use boost.
    #include <boost/date_time/posix_time/posix_time.hpp>
    #include <boost/thread/thread.hpp>
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
    
    // sleep functions
    static void msleep(double milliseconds);
    
    // battery functions
    virtual void displayBatteryState(NUSensorsData* data, NUActionatorsData* actions);
    // network functions
    virtual void displayTeamPacketReceived(NUActionatorsData* actions);
    virtual void displayTeamPacketSent(NUActionatorsData* actions);
    virtual void displayGamePacketReceived(NUActionatorsData* actions);
    virtual void displayOtherPacketReceived(NUActionatorsData* actions);
    // watchdog functions
    virtual void displayVisionFrameDrop(NUActionatorsData* actions);
    
    virtual std::string getWirelessMacAddress();
    virtual std::string getWiredMacAddress();
    
    virtual void restart() {};
private:
    // System time members
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        static struct timespec m_gettime_starttime;            //!< the program's start time according to gettime()
        static struct timespec m_gettimefast_starttime;        //!< the program's start time according to the fast verion of gettime()
    #else
        static boost::posix_time::ptime m_microsec_starttime;  //!< the program's start time according to boost::posix_time
    #endif
    static long double m_time_offset;                          //!< an offset so that timesincestart = unixstamp - offset and unixstamp = timesincestart + offset
};

extern NUSystem* nusystem;           //!< This is an omnipresent variable for the underlying system. Having a global variable is a compromise, hopefully it will be only exception @relates NUSystem

#endif

