/*! @file NUPlatform.cpp
    @brief Partial implementation of base NUPlatform (Robot) class

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
#include "NUPlatform.h"
#include "NUSensors.h"
#include "NUActionators.h"
#include "NUCamera.h"

#include "Infrastructure/NUBlackboard.h"
#include "Motion/Tools/MotionFileTools.h"

#ifndef CLOCK_REALTIME_FAST                             // not all distros will have the CLOCK_REALTIME_FAST, fudge it if they don't
    #define CLOCK_REALTIME_FAST CLOCK_REALTIME
#endif

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "nubotdataconfig.h"
#include "targetconfig.h"

#include <unistd.h>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <memory>
#ifdef TARGET_OS_IS_WINDOWS
    #include <windows.h>
#else
    #include <sys/ioctl.h>
    #include <sys/socket.h>
    #include <net/if.h>
    #include <netinet/in.h>
#endif

using namespace std;

NUPlatform* Platform = NULL;

NUPlatform::NUPlatform()
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 1
        debug << "NUPlatform::NUPlatform()" << endl;
    #endif
    Platform = this;
}

NUPlatform::~NUPlatform()
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 0
        debug << "NUPlatform::~NUPlatform()" << endl;
    #endif
    delete m_camera;
    m_camera = 0;
    delete m_sensors;
    m_sensors = 0;
    delete m_actionators;
    m_actionators = 0;
}

/*! @brief Initialises the NUPlatform's members 
    @note Every derived class must call this function inside its constructor. 
          This is necessary to properly call derived initName, initNumber, initTeam and initMAC
 */
void NUPlatform::init()
{
    initClock();
    initIdentity();
    debug << "NUPlatform Name: " << m_name << " Number: " << m_robot_number << " Team: " << m_team_number << " MAC: " << m_mac_address << endl;
}

/*! @brief Initialises the NUPlatform's clock 
    Sets m_gettime_starttime, m_micro_sec_starttime and m_time_offset 
 */
void NUPlatform::initClock()
{
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        clock_gettime(CLOCK_REALTIME_FAST, &m_gettime_starttime);
    #else
        m_microsec_starttime = boost::posix_time::microsec_clock::local_time();
    #endif
    m_time_offset = getPosixTimeStamp();
}

/*! @brief Initialises the NUPlatform's identity
    Sets m_name, m_robot_number, m_team_number, and m_mac_address
 */
void NUPlatform::initIdentity()
{
    initName();
    initNumber();
    initTeam();
    initMAC();
}

/*! @brief Initialises the NUPlatform's name
    Sets m_name. This is the default implementation for getting the robot's name, this function 
    is left as virtual for other platforms to define a specific implementation
 */
void NUPlatform::initName()
{
    // By default the robot's name should be the hostname
    char hostname[255];
    gethostname(hostname, 255);
    m_name = string(hostname); 
}

/*! @brief Initialises the NUPlatform's robot number
    Sets m_robot_number. This is the default implementation for getting the robot's number, this function 
    is left as virtual for other platforms to define a specific implementation
 */
void NUPlatform::initNumber()
{
    // By default the robot's number should be the last few characters of the name
    int robot_number_start = 0;
    for (size_t i=m_name.size()-1; i>0; i--)
    {
        if (isalpha(m_name[i]))
        {   // find the last alphabet, everything after that is the robot number
            robot_number_start = i;
            break;
        }
    }
    m_robot_number = atoi(m_name.substr(robot_number_start).c_str());
}

/*! @brief Initialises the NUPlatform's team number
    Sets m_team_number. This is the default implementation for getting the robot's team number, this function 
    is left as virtual for other platforms to define a specific implementation
 */
void NUPlatform::initTeam()
{
    // By default the team number is stored in a config file
    ifstream teamfile((string(CONFIG_DIR) + string("Team.cfg")).c_str());      // the team number is stored in a file
    if (teamfile.is_open())
        m_team_number = MotionFileTools::toFloat(teamfile);
    else
    {
        errorlog << "NUPlatform::initTeam(). Unable to load Team.cfg" << endl;
        m_team_number = 0;
    }
    teamfile.close();
}

/*! @brief Initialises the NUPlatform's mac address
    Sets m_mac_address. This is the default implementation for getting the robot's wired mac address, this function 
    is left as virtual for other platforms to define a specific implementation
 */
void NUPlatform::initMAC()
{
    // By default we use ioctl to get the wired MAC address
    #ifndef TARGET_OS_IS_WINDOWS
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1) 
            errorlog << "NUPlatform::initMAC(). Unable to open socket, so unable to get mac address." << endl;
        else
        {
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            #ifdef TARGET_OS_IS_DARWIN
                strcpy(ifr.ifr_name, "en0");
                if (ioctl(sockfd, SIOCGIFMAC, &ifr) != -1)
                {
                    // doesnt work on OS-X :(
                }
            #else
                strcpy(ifr.ifr_name, "eth0");
                bool success = ioctl(sockfd, SIOCGIFHWADDR, &ifr) != -1;
                if (not success)
                {   // if we fail to get the wired address, try to get the wireless
                    strcpy(ifr.ifr_name, "wlan0");
                    success = ioctl(sockfd, SIOCGIFHWADDR, &ifr) != -1;
                }
                if (success)
                {   // convert the ifr.ifr_hwaddr.sa_data to "XX-XX-XX-XX-XX"
                    stringstream ss;
                    ss << hex << setw(2) << setfill('0');
                    for (int i=0; i<5; i++)
                        ss << setw(2) << setfill('0') << (int) (unsigned char) ifr.ifr_hwaddr.sa_data[i] << "-";
                    ss << setw(2) << setfill('0') << (int) (unsigned char) ifr.ifr_hwaddr.sa_data[5];
                    m_mac_address = ss.str();
                }
            #endif
            close(sockfd);
        }
    #endif
}

/*! @brief Returns the name of the robot */
string& NUPlatform::getName()
{
    return m_name;
}

/*! @brief Returns the robot's number */
int NUPlatform::getRobotNumber()
{
    return m_robot_number;
}

/*! @brief Returns the robot's team number */
int NUPlatform::getTeamNumber()
{
    return m_team_number;
}

/*! @brief Returns the robot's mac address. */
string& NUPlatform::getMacAddress()
{
    return m_mac_address;
}

/*! @brief Returns a timestamp in milliseconds since the epoch (ie. The UNIX timestamp)

    In simulators, the timestamp will be fudged so that the timestamp returned will be the unix
    timestamp at the start of the simulation + the *simulated* time
 */
long double NUPlatform::getPosixTimeStamp()
{
    long double timeinmilliseconds;
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        struct timespec timenow;
        clock_gettime(CLOCK_REALTIME_FAST, &timenow);
        timeinmilliseconds = timenow.tv_nsec/1e6 + timenow.tv_sec*1e3;
    #else
        timeinmilliseconds = (boost::posix_time::microsec_clock::universal_time() - boost::posix_time::from_time_t(0)).total_nanoseconds()/1e6;
    #endif
    return timeinmilliseconds;
}

/*! @brief Returns the time in milliseconds since the start of the program
 
    For simulated robots this function returns the simulated time since the start of the program.
    For real robots this functions returns the real time since the start of the program.
    The getRealTime() function is provided which always returns the real time, use this for profiling code.
 */
double NUPlatform::getTime()
{
    return getRealTime();       // the default implementation is to just return the actual time
}

/*! @brief Returns the real time in milliseconds since the start of the program.
 
    This always returns the real time. Use this function to profile code.
 */
double NUPlatform::getRealTime()
{
    double timeinmilliseconds;
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        struct timespec timenow;
        clock_gettime(CLOCK_REALTIME_FAST, &timenow);
        timeinmilliseconds = (timenow.tv_nsec - m_gettime_starttime.tv_nsec)/1e6 + (timenow.tv_sec - m_gettime_starttime.tv_sec)*1e3;
    #else
        boost::posix_time::ptime timenow;
        timenow = boost::posix_time::microsec_clock::local_time();
        timeinmilliseconds = (timenow - m_microsec_starttime).total_nanoseconds()/1e6;
    #endif
    return timeinmilliseconds;
}

/*! @brief Returns the the time in milliseconds spent in this process since some arbitary point in the past. 
 
    This function might return the thread time if the platform doesn't support the process times. 
 */
double NUPlatform::getProcessTime()
{
    double timeinmilliseconds;
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        struct timespec timenow;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timenow);
        timeinmilliseconds = timenow.tv_nsec/1e6 + timenow.tv_sec*1e3;
    #else
        timeinmilliseconds = 1e3*clock()/float(CLOCKS_PER_SEC);
    #endif
    return timeinmilliseconds;
}

/*! @brief Returns the the time in milliseconds spent in this thread since the program started.  
 
    This function might return the process time if the platform doesn't support the thread times. 
 */
double NUPlatform::getThreadTime()
{
    double timeinmilliseconds;
    #ifdef __NU_SYSTEM_CLOCK_GETTIME
        struct timespec timenow;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timenow);
        timeinmilliseconds = timenow.tv_nsec/1e6 + timenow.tv_sec*1e3;
    #else
        timeinmilliseconds = 1e3*clock()/float(CLOCKS_PER_SEC);
    #endif
    return timeinmilliseconds;
}

/*! @brief Sleeps the calling thread for the specified time in milliseconds 
    @param milliseconds the duration of the sleep
 */
void NUPlatform::msleep(double milliseconds)
{
    #ifdef __NU_PERIODIC_CLOCK_NANOSLEEP
        struct timespec sleeptime;
        sleeptime.tv_sec = static_cast<int> (milliseconds/1e3);
        sleeptime.tv_nsec = 1e6*milliseconds - sleeptime.tv_sec*1e9;
        clock_nanosleep(CLOCK_REALTIME, 0, &sleeptime, NULL);  
    #else
        #ifdef TARGET_OS_IS_WINDOWS
            Sleep(DWORD(milliseconds));
        #else
            if (milliseconds <= 1000)
                usleep(static_cast<int> (milliseconds*1e3));
            else
            {
                sleep(milliseconds/1e3);
            }
        #endif
    #endif
}

/*! @brief Gets the pointer to the NUSensorsData object used by the platform to store sensor data */
NUSensorsData* NUPlatform::getNUSensorsData()
{
    return m_sensors->getNUSensorsData();
}

/*! @brief Gets the pointer to the NUActionatorsData object used by the platform to store actions for the hardware */
NUActionatorsData* NUPlatform::getNUActionatorsData()
{
    return m_actionators->getNUActionatorsData();
}

/*! @brief Updates the image in the Blackboard with a new one */
void NUPlatform::updateImage()
{
    Blackboard->Image = m_camera->grabNewImage();
}

/*! @brief Updates the sensor data in the Blackboard with new values */
void NUPlatform::updateSensors()
{
    m_sensors->update();
}

/*! @brief Processes the actions in the Blackboard */
void NUPlatform::processActions()
{
    m_actionators->process(Blackboard->Actions);
}

/*! @brief Kills the NUPlatform */
void NUPlatform::kill()
{
    if (m_actionators)
        m_actionators->process(Blackboard->Actions);
}

