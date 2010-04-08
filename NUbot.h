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

#include "NUPlatform/NUPlatform.h"

#ifdef USE_VISION
    #include "Vision/FieldObjects/FieldObjects.h"
    #include "Tools/Image/NUimage.h"
    #include "Vision/Vision.h"
#endif

#ifdef USE_LOCALISATION
    //#include "Localisation/Localisation.h"
#endif

#ifdef USE_BEHAVIOUR
    #include "Behaviour/Behaviour.h"
#endif

#ifdef USE_MOTION
    #include "Motion/NUMotion.h"
#endif

#include "NUPlatform/NUIO.h"

class NUSensorsData;
class NUActionatorsData;
class JobList;
class GameInformation;
class TeamInformation;

#if defined(USE_VISION) or defined(USE_LOCALISATION) or defined(USE_BEHAVIOUR) or defined(USE_MOTION)
    class SeeThinkThread;
#endif

class SenseMoveThread;

/*! @brief The top-level class
 */
class NUbot
{
public:
    NUbot(int argc, const char *argv[]);
    ~NUbot();
    void run();
    
private:
    void connectErrorHandling();
    static void segFaultHandler(int value);
    void unhandledExceptionHandler(std::exception& e);
    
    void createThreads();
    
public:
    #ifdef USE_VISION
        NUimage* Image;
    #endif
    NUSensorsData* SensorData;
    NUActionatorsData* Actions;
    JobList* Jobs;
    GameInformation* GameInfo;
    TeamInformation* TeamInfo;
    
private:
    NUPlatform* m_platform;               //!< interface to robot platform
    #ifdef USE_VISION
        Vision* m_vision;                 //!< vision module
    #endif
    
    #ifdef USE_LOCALISATION
        //Localisation* m_localisation;     //!< localisation module
    #endif
    
    #ifdef USE_BEHAVIOUR
        Behaviour* m_behaviour;           //!< behaviour module
    #endif
    
    #ifdef USE_MOTION
        NUMotion* m_motion;               //!< motion module
    #endif
    
    NUIO* m_io;                           //!< io module
    
    friend class SeeThinkThread;
    #if defined(USE_VISION) or defined(USE_LOCALISATION) or defined(USE_BEHAVIOUR) or defined(USE_MOTION)
        SeeThinkThread* m_seethink_thread;
    #endif

    friend class SenseMoveThread;
    SenseMoveThread* m_sensemove_thread;
    #if defined(TARGET_IS_NAO)
        friend class NUNAO;
    #endif
};

#endif

