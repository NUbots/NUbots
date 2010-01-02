/*! @file NUbot.cpp
    @brief Implementation of top-level NUbot class

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

#include <time.h>
#include <errno.h>
#include <iostream>
using namespace std;

#include "NUbot.h"
#ifdef TARGET_IS_NAOWEBOTS
    #include "NUPlatform/Platforms/NAOWebots/NAOWebotsPlatform.h"
#endif
#ifdef TARGET_IS_NAO
    #include "NUPlatform/Platforms/NAO/NAOPlatform.h"
#endif
#ifdef TARGET_IS_CYCLOID
    #include "NUPlatform/Platforms/Cycloid/CycloidPlatform.h"
#endif

static pthread_mutex_t mutexMotionData;    //!< lock for new motion data signal @relates NUbot
static pthread_cond_t condMotionData;      //!< signal for new motion data      @relates NUbot
static pthread_mutex_t mutexVisionData;    //!< lock for new vision data signal @relates NUbot
static pthread_cond_t condVisionData;      //!< signal for new vision data      @relates NUbot

static pthread_mutex_t mutexMotionRunning;           //!< in webots I need to use a mutex to prevent starting a motion iteration before the current one has completed @relates NUbot
static pthread_mutex_t mutexVisionRunning;           //!< in webots I need to use a mutex to prevent starting a vision iteration before the current one has completed @relates NUbot

static void* runThreadMotion(void* arg);
static void* runThreadVision(void* arg);

/*! @brief Constructor for the nubot
    
    The parameters are for command line arguements. Webots gives the binary arguements which tell us the 
    robot's number. Consequently, these args are passed down to the webots platform.
 
    @param argc the number of command line arguements
    @param *argv[] the array of command line arguements
 */
NUbot::NUbot(int argc, const char *argv[])
{
#if DEBUG_NUBOT_VERBOSITY > 4
    debug << "NUbot::NUbot(). Constructing NUPlatform." << endl;
#endif
    // Construct the right Platform
    #ifdef TARGET_IS_NAOWEBOTS
        platform = new NAOWebotsPlatform(argc, argv);
    #endif
    #ifdef TARGET_IS_NAO
        platform = new NAOPlatform();
    #endif
    #ifdef TARGET_IS_CYCLOID
        platform = new CycloidPlatform();
    #endif

#if DEBUG_NUBOT_VERBOSITY > 4
    debug << "NUbot::NUbot(). Constructing modules." << endl;
#endif
    
    // Construct each enabled module 
    #ifdef USE_VISION
        vision = new Vision();
    #endif
    #ifdef USE_LOCALISATION
        localisation = new Localisation();
    #endif
    #ifdef USE_BEHAVIOUR
        behaviour = new Behaviour();
    #endif
    #ifdef USE_MOTION
        motion = new NUMotion();
    #endif
    #ifdef USE_NETWORK
        network = new Network();
    #endif
    
    createThreads();
    
#if DEBUG_NUBOT_VERBOSITY > 4
    debug << "NUbot::NUbot(). Finished." << endl;
#endif
}

/*! @brief Create nubot's threads
 
    There is a single real-time thread in which motion runs. A semaphore is set by the body driver which triggers
    the execution of this thread. The fresh sensors data is processed by motion, and commands are sent to the actionators.
 
    The scheduling policy is SCHED_FIFO if the priority (THREAD_MOTION_PRIORITY) is non-zero, otherwise the default policy
    is selected. I recommend that this thread have a non-zero priority.
 
    There is a single non-real-time thread in which vision, localisation and behaviour run. A semaphore is set by the camera
    driver which triggers the execution of this thread. The new image is processed, and computed actions are sent to motion
    and lcs.
 
    The scheduling policy is SCHED_FIFO if the priority (THREAD_VISION_PRIORITY) is non-zero, otherwise the default policy
    is selected. I recommend that this thread have a zero priority.
 */

void NUbot::createThreads()
{
#if DEBUG_NUBOT_VERBOSITY > 4
    debug << "NUbot::createThreads(). Constructing threads." << endl;
#endif
    // create threadMotion, its trigger condMotionData and the running mutex mutexMotionRunning
    int err;
    
    err = pthread_mutex_init(&mutexMotionData, NULL);
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to initialise mutexMotionData errno: " << errno << endl;
    
    err = pthread_cond_init(&condMotionData, NULL);
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to initialise condMotionData errno: " << errno << endl;
    
    err = pthread_mutex_init(&mutexMotionRunning, NULL);
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to initialise mutexMotionRunning errno: " << errno << endl;
    
#if THREAD_MOTION_PRIORITY > 0
    err = pthread_create(&threadMotion, NULL, runThreadMotion, (void*) this);         // The last parameter is the arguement to the thread
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to create threadMotion! The error code was: " << err << endl;
    
    sched_param param;
    param.sched_priority = THREAD_MOTION_PRIORITY;
    pthread_setschedparam(threadMotion, SCHED_FIFO, &param);            // Note. This will fail (quietly) if the underlying OS doesn't allow real-time
    
    // double check
    int actualpolicy;
    sched_param actualparam;
    pthread_getschedparam(threadMotion, &actualpolicy, &actualparam);
    #if DEBUG_NUBOT_VERBOSITY > 4
        debug << "NUbot::createThreads(). threadMotion Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << endl;
    #endif
#else   
    debug << "NUbot::createThreads(). Warning. Creating threadMotion as non-realtime" << endl;
    int err = pthread_create(&threadMotion, NULL, runThreadMotion, (void*) this);
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to create threadMotion! The error code was: " << err << endl;
#endif

    // create threadVision, its trigger condVisionData and the running mutex mutexVisionRunning
    err = pthread_mutex_init(&mutexVisionData, NULL);
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to initialise mutexVisionData errno: " << errno << endl;
    
    err = pthread_cond_init(&condVisionData, NULL);
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to initialise condVisionData errno: " << errno << endl;
    
    err = pthread_mutex_init(&mutexVisionRunning, NULL);
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to initialise mutexVisionRunning errno: " << errno << endl;
    
#if THREAD_VISION_PRIORITY > 0
    debug << "NUbot::createThreads(). Warning. Creating threadVision as realtime" << endl;
    err = pthread_create(&threadVision, NULL, runThreadVision, (void*) this);         // The last parameter is the arguement to the thread
    
    sched_param param;
    param.sched_priority = THREAD_VISION_PRIORITY;
    pthread_setschedparam(threadVision, SCHED_FIFO, &param);            // Note. This will fail (quietly) if the underlying OS doesn't allow real-time
    
    // double check
    int actualpolicy;
    sched_param actualparam;
    pthread_getschedparam(threadVision, &actualpolicy, &actualparam);
    #if DEBUG_NUBOT_VERBOSITY > 4
        debug << "NUbot::createThreads(). threadMotion Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << endl;
    #endif
#else   
    err = pthread_create(&threadVision, NULL, runThreadVision, (void*) this);
#endif
    if (err != 0)
        debug << "NUbot::createThreads(). Failed to create threadVision! The error code was: " << err << endl;
    
#if DEBUG_NUBOT_VERBOSITY > 4
    debug << "NUbot::createThreads(). Finished." << endl;
#endif
}

/*! @brief Destructor for the nubot
 */
NUbot::~NUbot()
{
    #if DEBUG_NUBOT_VERBOSITY > 4
        debug << "NUbot::~NUbot()." << endl;
    #endif
    
    // destroy threading
    pthread_cancel(threadVision);
    pthread_cancel(threadMotion);
    
    pthread_mutex_destroy(&mutexVisionData);
    pthread_cond_destroy(&condVisionData);
    pthread_mutex_destroy(&mutexVisionRunning);
    pthread_mutex_destroy(&mutexMotionData);
    pthread_cond_destroy(&condMotionData);
    pthread_mutex_destroy(&mutexMotionRunning);
    
    // delete modules
    delete platform;
    #ifdef USE_VISION
        delete vision;
    #endif
    #ifdef USE_LOCALISATION
        delete localisation;
    #endif
    #ifdef USE_BEHAVIOUR
        delete behaviour;
    #endif
    #ifdef USE_MOTION
        delete motion;
    #endif
    #ifdef USE_NETWORK
        delete network;
    #endif
    #if DEBUG_NUBOT_VERBOSITY > 4
        debug << "NUbot::~NUbot(). Finished." << endl;
    #endif
}

/*! @brief The nubot's main loop
    
    The nubot's main loop. This function will probably never return.
 
    The idea is to simply have 
    @verbatim
    NUbot* nubot = new NUbot(arc, argv);
    nubot->run();
    delete nubot;
    @endverbatim

 */
void NUbot::run()
{
#ifdef TARGET_IS_NAOWEBOTS
    int count = 0;
    NAOWebotsPlatform* webots = (NAOWebotsPlatform*) platform;
    while (true)
    {
        webots->step(40);           // stepping the simulator generates new data to run motion, and sometimes the vision data
        debug << "NUbot::run()" << endl;
        signalMotion();
        if (count%2 == 0)           // depending on the selected frame rate vision might not need to be updated every simulation step
        {
            signalVision();         
            waitForVisionCompletion();
        }
        waitForMotionCompletion();     
        count++;
    };
#endif
}

/*! @brief Signal the motion thread to run
 
    This should be called by the underlying platform when new motion data is avaliable.
    It is implemented using a pthread_mutex_t and a pthread_cond_t.
 
    This function is static so that the underlying platform does not need a pointer to the nubot.
    However, this means the pthread_mutex_t and pthread_cond_t are external to the NUbot class.
 
    @return returns the error return by the underlying pthread_cond_signal
 */
int NUbot::signalMotion()
{
    int err = 0;
    pthread_mutex_lock(&mutexMotionData);
    err = pthread_cond_signal(&condMotionData);
    pthread_mutex_unlock(&mutexMotionData);
    return err;
}

/*! @brief Blocks the calling thread until signalMotion() is called
 
    @return returns the error return by the underlying pthread_cond_wait
 */
int NUbot::waitForNewMotionData()
{
    int err = 0;
    pthread_mutex_lock(&mutexMotionData);
    err = pthread_cond_wait(&condMotionData, &mutexMotionData);
    pthread_mutex_unlock(&mutexMotionData);
    return err;
}

/*! @brief Signal that the motion thread has completed
 
 @return returns the error return by the underlying pthread_cond_signal
 */
int NUbot::signalMotionStart()
{
    int err = 0;
    err = pthread_mutex_lock(&mutexMotionRunning);
    return err;
}

/*! @brief Signal that the motion thread has completed
 
 @return returns the error return by the underlying pthread_cond_signal
 */
int NUbot::signalMotionCompletion()
{
    int err = 0;
    err = pthread_mutex_unlock(&mutexMotionRunning);
    return err;
}

/*! @brief Blocks the calling thread until motion thread completes its iteration
 
 @return returns the error return by the underlying pthread_cond_wait
 */
int NUbot::waitForMotionCompletion()
{
    int err = 0;
    err = pthread_mutex_lock(&mutexMotionRunning);            // block if motion thread is STILL running
    pthread_mutex_unlock(&mutexMotionRunning);
    return err;
}

/*! @brief Signal the vision thread to run
 
 This should be called by the underlying platform when new vision data is avaliable.
 It is implemented using a pthread_mutex_t and a pthread_cond_t.
 
 This function is static so that the underlying platform does not need a pointer to the nubot.
 However, this means the pthread_mutex_t and pthread_cond_t are external to the NUbot class.
 
 @return returns the error return by the underlying pthread_cond_signal
 */
int NUbot::signalVision()
{
    int err = 0;
    pthread_mutex_lock(&mutexVisionData);
    pthread_cond_signal(&condVisionData);
    pthread_mutex_unlock(&mutexVisionData);
    return err;
}

/*! @brief Blocks the calling thread until signalVision() is called
 
 @return returns the error return by the underlying pthread_cond_wait
 */
int NUbot::waitForNewVisionData()
{
    int err = 0;
    pthread_mutex_lock(&mutexVisionData);
    err = pthread_cond_wait(&condVisionData, &mutexVisionData);
    pthread_mutex_unlock(&mutexVisionData);
    return err;
}

/*! @brief Signal that the vision thread has started an iteration
 
 @return returns the error return by the underlying pthread_mutex
 */
int NUbot::signalVisionStart()
{
    int err = 0;
    err = pthread_mutex_lock(&mutexVisionRunning);
    return err;
}

/*! @brief Signal that the vision thread has completed
 
 @return returns the error return by the underlying pthread_mutex
 */
int NUbot::signalVisionCompletion()
{
    int err = 0;
    err = pthread_mutex_unlock(&mutexVisionRunning);
    return err;
}

/*! @brief Blocks the calling thread until vision thread completes its iteration
 
 @return returns the error return by the underlying pthread_cond_wait
 */
int NUbot::waitForVisionCompletion()
{
    int err = 0;
    err = pthread_mutex_lock(&mutexVisionRunning);
    pthread_mutex_unlock(&mutexVisionRunning);
    return err;
}

/*! @brief The motion control loop
    @relates NUbot
 
    The thread is set up to run when signalled by signalMotion(). It will quickly grab the
    new sensor data, compute a response, and then send the commands to the actionators.
 
    @param arg a pointer to the nubot
 */
void* runThreadMotion(void* arg)
{
    debug << "NUbot::runThreadMotion: Starting." << endl;
    
    NUbot* nubot = (NUbot*) arg;                
    NUSensorsData* data = NULL;
    NUActionatorsData* actions = NULL;
    
#ifdef THREAD_MOTION_MONITOR_TIME
    double entrytime;
    double realstarttime, processstarttime, threadstarttime; 
    double realendtime, processendtime, threadendtime;
#endif
    
    int err;
    do 
    {
#if defined THREAD_MOTION_MONITOR_TIME and !defined TARGET_IS_NAOWEBOTS
        entrytime = NUSystem::getRealTime();
#endif
        err = nubot->waitForNewMotionData();
        nubot->signalMotionStart();

#ifdef THREAD_MOTION_MONITOR_TIME
    #ifndef TARGET_IS_NAOWEBOTS         // there is not point monitoring wait times in webots
        realstarttime = NUSystem::getRealTime();
        if (realstarttime - entrytime > 25)
            debug << "NUbot::runThreadMotion. Waittime " << realstarttime - entrytime << " ms."<< endl;
    #endif
        processstarttime = NUSystem::getProcessTime();
        threadstarttime = NUSystem::getThreadTime();
#endif
        
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        debug << "NUbot::runThreadMotion " << nubot->platform->system->getTime() << endl;
        data = nubot->platform->sensors->update();
        #ifdef USE_MOTION
            nubot->motion->process(data, actions);
        #endif
        nubot->platform->actionators->process(actions);
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef THREAD_MOTION_MONITOR_TIME
        realendtime = NUSystem::getRealTime();
        processendtime = NUSystem::getProcessTime();
        threadendtime = NUSystem::getThreadTime();
        if (threadendtime - threadstarttime > 3)
            debug << "NUbot::runThreadMotion. Thread took a long time to complete. Time spent in this thread: " << (threadendtime - threadstarttime) << "ms, in this process: " << (processendtime - processstarttime) << "ms, in realtime: " << realendtime - realstarttime << "ms." << endl;
#endif
        nubot->signalMotionCompletion();
    } 
    while (err == 0 | errno != EINTR);
    pthread_exit(NULL);
}

/*! @brief The vision control loop
    @relates NUbot
 
     The thread is set up to run when signalled by signalVision(). It will grab the
     new image, compute a response, and then send the commands to motion and lcs.
 
    @param arg a pointer to the nubot
 */
void* runThreadVision(void* arg)
{
    debug << "NUbot::runThreadVision: Starting." << endl;
    
    NUbot* nubot = (NUbot*) arg;                // the nubot
    //NUSensorsData* data = NULL;
    //NUActionatorsData* actions = NULL;
    JobList joblist = JobList();
    
    vector<float> walkspeed(3, 0);
    walkspeed[0] = 6;       // max 7cm/s
    walkspeed[1] = -0;
    walkspeed[2] = 0;
    
    joblist.addVisionJob(new WalkJob(walkspeed));
    
    
#ifdef THREAD_VISION_MONITOR_TIME
    double entrytime;
    double realstarttime, processstarttime, threadstarttime; 
    double realendtime, processendtime, threadendtime;
#endif
    
    double timenow;
    
    int err;
    do 
    {
#if defined THREAD_VISION_MONITOR_TIME && !defined TARGET_IS_NAOWEBOTS
        entrytime = NUSystem::getRealTimeFast();
#endif
        err = nubot->waitForNewVisionData();
        nubot->signalVisionStart();
        
#ifdef THREAD_VISION_MONITOR_TIME
    #ifndef TARGET_IS_NAOWEBOTS         // there is not point monitoring wait times in webots
        realstarttime = NUSystem::getRealTimeFast();
        if (realstarttime - entrytime > 1000/15.0 + 5)
            debug << "NUbot::runThreadVision. Waittime " << realstarttime - entrytime << " ms."<< endl;
    #endif
        processstarttime = NUSystem::getProcessTime();
        threadstarttime = NUSystem::getThreadTime();
#endif
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        debug << "NUbot::runThreadVision " << nubot->platform->system->getTime() << endl;
        //          image = nubot->platform->camera->getData()
        //          data = nubot->platform->sensors->getData()                // I should not deep copy the data here
        //                 odometry = nubot->motion->getData()                // There is no deep copy here either
        //      gamectrl, teaminfo = nubot->network->getData()
        //          fieldobj = nubot->vision->process(image, data, gamectrl)
        //          wm = nubot->localisation->process(fieldobj, teaminfo, odometry, gamectrl, actions)
        nubot->behaviour->process(joblist);      //TODO: nubot->behaviour->process(wm, gamectrl, p_jobs)
        
        if (nusystem->getTime() > 0)
            joblist.addMotionJob(new WalkJob(walkspeed));
        #ifdef USE_MOTION
            nubot->motion->process(joblist);
        #endif
        //          cmds = nubot->lcs->process(lcsactions)
        //nubot->platform->actionators->process(m_actions);
        //joblist.clear();                           // assume that all of the jobs have been completed
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef THREAD_VISION_MONITOR_TIME
        realendtime = NUSystem::getRealTimeFast();
        processendtime = NUSystem::getProcessTime();
        threadendtime = NUSystem::getThreadTime();
        if (threadendtime - threadstarttime > 10)
            debug << "NUbot::runThreadVision. Thread took a long time to complete. Time spent in this thread: " << (threadendtime - threadstarttime) << "ms, in this process: " << (processendtime - processstarttime) << "ms, in realtime: " << realendtime - realstarttime << "ms." << endl;
#endif
        nubot->signalVisionCompletion();
    } 
    while (err == 0 | errno != EINTR);
    pthread_exit(NULL);
}

