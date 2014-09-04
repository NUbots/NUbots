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

#include "ConfigSystem/Module.h" // CONFIG_SYSTEM_TEST

#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"
#include "NUPlatform/NUIO.h"
#include "NUbot.h"
#include "SeeThinkThread.h"
#include "nubotdataconfig.h"

//#define LOGGING_ENABLED 1

#ifdef USE_VISION
    #include "Infrastructure/FieldObjects/FieldObjects.h"
    #include "Infrastructure/NUImage/NUImage.h"
    #include "Vision/VisionWrapper/visioncontrolwrapperdarwin.h"
#endif

#ifdef USE_BEHAVIOUR
    #include "Behaviour/Behaviour.h"
    #include "Infrastructure/Jobs/Jobs.h"
#endif

#ifdef USE_LOCALISATION
    #include "Localisation/SelfLocalisation.h"
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

SeeThinkThread::SeeThinkThread(NUbot* nubot) : ConditionalThread(std::string("SeeThinkThread"), THREAD_SEETHINK_PRIORITY)
{
    #if DEBUG_VERBOSITY > 0
        debug << "SeeThinkThread::SeeThinkThread(" << nubot << ") with priority " << static_cast<int>(m_priority) << std::endl;
    #endif
    m_nubot = nubot;
    m_logrecorder = new LogRecorder(m_nubot->m_blackboard->GameInfo->getPlayerNumber());
#ifdef LOGGING_ENABLED
    m_logrecorder->SetLogging("sensor",true);
    m_logrecorder->SetLogging("gameinfo",true);
    m_logrecorder->SetLogging("teaminfo",true);
    m_logrecorder->SetLogging("object",true);
    m_logrecorder->SetLogging("image",true);
#endif
}

SeeThinkThread::~SeeThinkThread()
{
    #if DEBUG_VERBOSITY > 0
        debug << "SeeThinkThread::~SeeThinkThread()" << std::endl;
    #endif
    stop();
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
        debug << "SeeThinkThread::run()" << std::endl;
    #endif
    #ifdef THREAD_SEETHINK_PROFILE
        Profiler prof = Profiler("SeeThinkThread");
    #endif
#ifdef LOGGING_ENABLED
    ofstream locfile((std::string(DATA_DIR) + std::string("selflocwm.strm")).c_str(), ios_base::trunc);
#endif
    int err = 0;
    while (err == 0 && errno != EINTR)
    {
        try
        {
            #if defined(TARGET_IS_NAOWEBOTS) or defined(TARGET_IS_DARWINWEBOTS) or (not defined(USE_VISION))
                wait();
            #endif
            
            // ---- Update the configuration system ----
            // // Note: should add a define instead of just 
            // //       commenting/uncommenting this all the time.
            // // #ifdef SOME_CONFIG_TESTING_DEFINE
            // if(Module::autoUpdateTest()) // CONFIG_SYSTEM_TEST
            //     std::cout << "SeeThinkThread::run(): autoUpdateTest Success!" << std::endl;
            // else 
            //     std::cout << "SeeThinkThread::run(): autoUpdateTest FAIL!" << std::endl;
            // // #endif
            Blackboard->Config->UpdateConfiguration();
            // -----------------------------------------

            #ifdef THREAD_SEETHINK_PROFILE
                prof.start();
            #endif
            #ifdef USE_VISION
                m_nubot->m_platform->updateImage();
                *(m_nubot->m_io) << m_nubot;  //<! Raw IMAGE STREAMING (TCP)
            #endif
            
            #ifdef THREAD_SEETHINK_PROFILE
                prof.split("frame grab");
            #endif
            // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
            #ifdef USE_VISION
                m_nubot->m_vision->runFrame();
                #ifdef THREAD_SEETHINK_PROFILE
                    prof.split("vision");
                #endif
            #endif

            double current_time = Blackboard->Sensors->GetTimestamp();
            Blackboard->TeamInfo->UpdateTime(current_time);
            Blackboard->GameInfo->UpdateTime(current_time);
            m_logrecorder->WriteData(Blackboard);

            #ifdef THREAD_SEETHINK_PROFILE
                prof.split("time update");
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
                #ifdef THREAD_SEETHINK_PROFILE
                    prof.split("debug print job summary");
                #endif
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

					
            //std::cout << m_nubot->m_platform->getRealTime() << std::endl << Blackboard->Image->GetTimestamp() << std::endl << std::endl;
            m_nubot->m_api->sendAll();
			
#ifdef LOGGING_ENABLED
            locfile << *m_nubot->m_localisation;
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
    errorlog << "SeeThinkThread is exiting. err: " << err << " errno: " << errno << std::endl;
}
