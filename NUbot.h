/*! @file NUbot.h
    @brief Declaration of top-level NUbot class.

    @mainpage
    The NUbot's RoboCup code doxygen documentation. This software makes robots (NAO, NAOWebots and Cycloid)
    autonomously play soccer.
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef NUBOT_H
#define NUBOT_H

#include "targetconfig.h"
#include "nubotconfig.h"

class NUBlackboard;
class NUPlatform;
class NUIO;

#ifdef USE_VISION
    class Vision;
#endif

#ifdef USE_LOCALISATION
    class Localisation;
#endif

#ifdef USE_BEHAVIOUR
    class Behaviour;
#endif

#ifdef USE_MOTION
    class NUMotion;
#endif

#if defined(USE_VISION) or defined(USE_LOCALISATION) or defined(USE_BEHAVIOUR) or defined(USE_MOTION)
    class SeeThinkThread;
#endif
class SenseMoveThread;
class WatchDogThread;

#include <exception>

/*! @brief The top-level class
 */
class NUbot
{
public:
    NUbot(int argc, const char *argv[]);
    ~NUbot();
    void run();
#ifdef USE_LOCALISATION
    const Localisation* GetLocWm(){return m_localisation;};
#endif
    
private:
    void createErrorHandling();
    void createPlatform(int argc, const char *argv[]);
    void destroyPlatform();
    void createBlackboard();
    void destroyBlackboard();
    void createNetwork();
    void destroyNetwork();
    void createModules();
    void destroyModules();
    void createThreads();
    void destroyThreads();
    
    void periodicSleep(int period);
    static void terminationHandler(int signum);
    void unhandledExceptionHandler(std::exception& e);
private:
    static NUbot* m_this;                 //!< a pointer to the last instance of a NUbot
    NUPlatform* m_platform;               //!< interface to robot platform
    NUBlackboard* m_blackboard;           //!< a pointer to the public store
    #ifdef USE_VISION
        Vision* m_vision;                 //!< vision module
    #endif
    
    #ifdef USE_LOCALISATION
        Localisation* m_localisation;     //!< localisation module
    #endif
    
    #ifdef USE_BEHAVIOUR
        Behaviour* m_behaviour;           //!< behaviour module
    #endif
    
    #ifdef USE_MOTION
        NUMotion* m_motion;               //!< motion module
    #endif
    
    NUIO* m_io;                           //!< io module
public:         //! @todo TODO: this door should be closed. It is open atm because I need the sensemove_thread to connect to the serial comms of the bear.
    friend class SeeThinkThread;
    #if defined(USE_VISION) or defined(USE_LOCALISATION)
        SeeThinkThread* m_seethink_thread;
    #endif

    friend class SenseMoveThread;
    SenseMoveThread* m_sensemove_thread;
    #if defined(TARGET_IS_NAO)
        friend class NUNAO;
    #endif
    
    friend class WatchDogThread;
    WatchDogThread* m_watchdog_thread;
};

#endif

