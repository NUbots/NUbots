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
#include "Motion/Walks/walkconfig.h"
#include "Tools/debug.h"

#include "NUPlatform/NUPlatform.h"
// Selectively include modules depending on targetconfig.h
#ifdef USE_VISION
    #include "Vision/Vision.h"
#endif

#ifdef USE_LOCALISATION
    #include "Localisation/Localisation.h"
#endif

#ifdef USE_BEHAVIOUR
    #include "Behaviour/Behaviour.h"
    #include "Behaviour/Jobs.h"
#endif

#ifdef USE_MOTION
    #include "Motion/NUMotion.h"
    #ifdef USE_WALKOPTIMISER
        #include "Motion/Walks/Optimisation/WalkOptimiserBehaviour.h"
    #endif
#endif

#include <pthread.h>

/*! @brief The top-level class
 */
class NUbot
{
// Functions:
public:
    NUbot(int argc, const char *argv[]);
    ~NUbot();
    void run();
    
    static int signalMotion();
    int signalMotionStart();
    int signalMotionCompletion();
    static int signalVision();
    int signalVisionStart();
    int signalVisionCompletion();
    
    int waitForNewMotionData();
    int waitForMotionCompletion();
    int waitForNewVisionData();
    int waitForVisionCompletion();
private:
    void createThreads();
    
public:
    NUPlatform* platform;               //!< interface to robot platform
    #ifdef USE_VISION
        Vision* vision;                 //!< vision module
    #endif
    #ifdef USE_LOCALISATION
        Localisation* localisation;     //!< localisation module
    #endif
    #ifdef USE_BEHAVIOUR
        Behaviour* behaviour;           //!< behaviour module
    #endif
    #ifdef USE_MOTION
        NUMotion* motion;               //!< motion module
        #ifdef USE_WALKOPTIMISER
            WalkOptimiserBehaviour* walkoptimiser;      //!< walk optimisation module
        #endif
    #endif
private:
    pthread_t threadMotion;             //!< thread containing the direct sensory links to motion (cerebellum)
    pthread_t threadVision;             //!< thread containing vision and higher-level though processes (cerebrum)
};

#endif

