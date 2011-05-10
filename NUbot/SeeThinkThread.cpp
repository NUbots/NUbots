/*! @file SeeThinkThread.cpp
    @brief Implementation of the see->think thread class.

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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



#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"
#include "NUPlatform/NUIO.h"
#include "NUbot.h"
#include "SeeThinkThread.h"
#include "Localisation/LocWmFrame.h"
#include "nubotdataconfig.h"


#ifdef USE_VISION
    #include "Infrastructure/FieldObjects/FieldObjects.h"
    #include "Infrastructure/NUImage/NUImage.h"
    #include "Vision/Vision.h"
#endif

#ifdef USE_BEHAVIOUR
    #include "Behaviour/Behaviour.h"
    #include "Infrastructure/Jobs/Jobs.h"
#endif

#ifdef USE_LOCALISATION
    #include "Localisation/Localisation.h"
#endif

#ifdef USE_MOTION
    #include "Motion/NUMotion.h"
#endif

#include "debug.h"
#include "debugverbositynubot.h"
#include "debugverbositythreading.h"

#ifdef THREAD_SEETHINK_PROFILE
    #include "Tools/Profiling/Profiler.h"
#endif

#include <errno.h>

#if DEBUG_NUBOT_VERBOSITY > DEBUG_THREADING_VERBOSITY
    #define DEBUG_VERBOSITY DEBUG_NUBOT_VERBOSITY
#else
    #define DEBUG_VERBOSITY DEBUG_THREADING_VERBOSITY
#endif

/*! @brief Constructs the sense->move thread
 */

SeeThinkThread::SeeThinkThread(NUbot* nubot) : ConditionalThread(string("SeeThinkThread"), THREAD_SEETHINK_PRIORITY)
{
    #if DEBUG_VERBOSITY > 0
        debug << "SeeThinkThread::SeeThinkThread(" << nubot << ") with priority " << static_cast<int>(m_priority) << endl;
    #endif
    m_nubot = nubot;


    m_locwmfile.open((string(DATA_DIR) + string("locfrm.strm")).c_str());
    debug << "Opening file: " << (string(DATA_DIR) + string("locfrm.strm")).c_str() << " ... ";
    if(m_locwmfile.is_open()) debug << "Success.";
    else debug << "Failed.";
    debug << std::endl;
}

SeeThinkThread::~SeeThinkThread()
{
    #if DEBUG_VERBOSITY > 0
        debug << "SeeThinkThread::~SeeThinkThread()" << endl;
    #endif
    stop();
    m_locwmfile.close();
}

/*! @brief The sense->move main loop
 
    When signalled the thread will quickly grab the new sensor data, compute a response, 
    and then send the commands to the actionators.
 
    Note that you can not safely use the job interface in this thread, if you need to add
    jobs provide a process function for this thread, and *another* process for the behaviour 
    thread which creates the jobs.
 
 */
void SeeThinkThread::run()
{
    #if DEBUG_VERBOSITY > 0
        debug << "SeeThinkThread::run()" << endl;
    #endif
    #ifdef THREAD_SEETHINK_PROFILE
        Profiler prof = Profiler("SeeThinkThread");
    #endif
    int err = 0;
    while (err == 0 && errno != EINTR)
    {
        try
        {
            #if defined(TARGET_IS_NAOWEBOTS) or (not defined(USE_VISION))
                wait();
            #endif
            #ifdef USE_VISION
                m_nubot->m_platform->updateImage();
                *(m_nubot->m_io) << m_nubot;  //<! Raw IMAGE STREAMING (TCP)
            #endif
            
            #ifdef THREAD_SEETHINK_PROFILE
                prof.start();
            #endif
            // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
            #ifdef USE_VISION
                m_nubot->m_vision->ProcessFrame(Blackboard->Image, Blackboard->Sensors, Blackboard->Actions, Blackboard->Objects);
                #ifdef THREAD_SEETHINK_PROFILE
                    prof.split("vision");
                #endif
            #endif

            #ifdef USE_LOCALISATION
                m_nubot->m_localisation->process(Blackboard->Sensors, Blackboard->Objects, Blackboard->GameInfo, Blackboard->TeamInfo);
                #ifdef THREAD_SEETHINK_PROFILE
                    prof.split("localisation");
                #endif
            #endif
            
            #if defined(USE_BEHAVIOUR)
                m_nubot->m_behaviour->process(Blackboard->Jobs, Blackboard->Sensors, Blackboard->Actions, Blackboard->Objects, Blackboard->GameInfo, Blackboard->TeamInfo);
                #ifdef THREAD_SEETHINK_PROFILE
                    prof.split("behaviour");
                #endif
            #endif
            
            #if DEBUG_VERBOSITY > 0
                Blackboard->Jobs->summaryTo(debug);
            #endif
            
            #ifdef USE_VISION

            m_nubot->m_vision->process(Blackboard->Jobs) ; //<! Networking for Vision
            m_nubot->m_platform->process(Blackboard->Jobs, m_nubot->m_io); //<! Networking for Platform

                #ifdef THREAD_SEETHINK_PROFILE
                    prof.split("vision_jobs");
                #endif
            #endif
            #ifdef USE_MOTION
                m_nubot->m_motion->process(Blackboard->Jobs);
                #ifdef THREAD_SEETHINK_PROFILE
                    prof.split("motion_jobs");
                #endif
            #endif
            // -----------------------------------------------------------------------------------------------------------------------------------------------------------------

            #ifdef THREAD_SEETHINK_PROFILE
                debug << prof;
            #endif
        }
        catch (std::exception& e)
        {
            m_nubot->unhandledExceptionHandler(e);
        }
    } 
    errorlog << "SeeThinkThread is exiting. err: " << err << " errno: " << errno << endl;
}
