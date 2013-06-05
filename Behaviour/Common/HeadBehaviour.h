/*! @file HeadBehaviour.h
    @brief Head behaviour class for simpler behaviour control.

    @author Josiah Walker

 Copyright (c) 2012 Josiah Walker

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
#ifndef HEADBEHAVIOUR_H
#define HEADBEHAVIOUR_H


#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "NUPlatform/NUCamera/NUCameraData.h"
#include "Tools/Math/General.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "nubotdataconfig.h"

//#include <boost/date_time/posix_time/posix_time.hpp>



//RLAgent import
#include "Tools/RLearning/MRLAgent.h"


#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "HeadLogic.h"
#include <cstdlib>
#include <ctime>

class HeadBehaviour {
public:
    enum VisionPolicyID{
        //Better Head Behaviours (are functions of basic ones)
        PrioritiseLocalisationPolicy,
        PrioritiseBallPolicy,
        LookAtBallPolicy,
        LookForBallPolicy,
        LookForFieldObjectsPolicy,
        //Basic Behaviours
        TimeVSCostPriority,//See above
        RLAgentTrainingPolicy,
        MRLAgentTrainingPolicy,//See above
        RLAgentPolicy,
        MRLAgentPolicy,
        CheckAgentPolicy //Checks the Agent's (both RL and MRL) policy without learning.
    };
private:

    int MAX_PERCEPT_RANGESIZE;

    /*MRLAgent:*/
    MRLAgent Mrlagent;

    /*! @brief Gets data from HeadLogic and compiles it into a single vector for feeding to reinforcement learning agent.
    */
    vector<float> getPercept();

    /*! @brief Calculates reward of current state.
    */
    float calculateReward();


    //These are the camera margins objects must be inside (as a percentage) when the robot looks at them.
    float cameraMarginX;
    float cameraMarginY;
    float m_CAMERA_FOV_X;
    float m_CAMERA_FOV_Y;
    
    //These control prioritising what to look at
    float ballSeenFrequency;
    float landmarkSeenFrequency;
    float maximumSearchTime;
    float CONFIRMATION_TIME;
    float ballFocusBias;
    
    //sets the time the action will finish so a new one can be chosen
    float actionStartTime;
    float actionEndTime;
    int actionObjectID;
    float buttonPressTime;
    float time_since_last_localisation;
    //holds the last vision policy used
    int lastVisionPolicy;
    VisionPolicyID current_policy;

    //variables for training
    int ACTIONS_PER_STATE;
    int actions_taken_this_state;
    string rewards_log_pathname;
    string agent_filename;

    float last_reward;
    

    /*! @brief
    */
    bool ObjectNotSeen();
    HeadBehaviour();
    ~HeadBehaviour();

    HeadLogic* head_logic;


    /*! @brief Perform a simple list policy for directing head behaviour.
    */
    void doPriorityListPolicy();

    /*! @brief Perform a policy which choice maximises time since last seen and minimises head movement cost.
    */
    void doTimeVSCostPriorityPolicy();

    /*! @brief Use the motivated reinforcement learning agent to make policy decisions.
    */
    void doAgentBasedPolicy();
    void doMRLAgentPolicy();
    void doRLAgentPolicy();
    void doCheckAgentPolicy();

    /*! @brief Writes to the HeadRewards.log file in nubot folder
    */
    void recordReward(float r);


public:


    
    static HeadBehaviour* getInstance();

    /*! @brief Method to call when making vision choice. Makes choice depending on policy chosen from those enumerated above.
    */
    void makeVisionChoice(VisionPolicyID fieldVisionPolicy);

    void prioritiseLocalisation();
    void prioritiseBall();
    void lookAtBall();
    void lookForBall();
    void lookForFieldObjects();
    void update();

    /*! @brief dispatchHeadJob methods. There are three, one for each object type.
    */
    void dispatchHeadJob(MobileObject *ObjectToTrack);
    
    void dispatchHeadJob(StationaryObject* ObjectToTrack);

    void dispatchHeadJob(AmbiguousObject* ObjectToTrack);
    
    //XXX: head jobs do not map from localisation space back to vision space properly, fix in here.
    /*void fieldXYToHeadingElevation(Vector2<float>* fieldposition, Vector2<float>* cameraposition) {
        
    }*/
    
    

};

#endif
