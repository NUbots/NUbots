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

#include "Tools/RLearning/MRLAgent.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "HeadLogic.h"

class HeadBehaviour {

private:
    //MRLAgent:
    MRLAgent Mrlagent;
    
    //These are the camera margins objects must be inside (as a percentage) when the robot looks at them.
    float cameraMarginX;
    float cameraMarginY;
    float m_CAMERA_FOV_X;
    float m_CAMERA_FOV_Y;
    
    //These control prioritising what to look at
    float ballSeenFrequency;
    float landmarkSeenFrequency;
    float maximumSearchTime;
    float ballFocusBias;
    
    //sets the time the action will finish so a new one can be chosen
    float actionStartTime;
    float actionEndTime;
    int actionObjectID;
    
    //holds the last vision policy used
    int lastVisionPolicy;
    

    
    bool ObjectNotSeen();
    HeadBehaviour();
    ~HeadBehaviour();

    HeadLogic* head_logic;
    void doPriorityListPolicy();
    void doTimeVSCostPriorityPolicy();
    void doRLAgentPolicy();

public:

    enum VisionPolicyID{
        BallFarVisionPolicy = 0,
        BallNearVisionPolicy = 1,
        BallLostVisionPolicy = 2,
        RobotLostVisionPolicy = 3,
        BallOnlyVisionPolicy = 4,
        LandmarkOnlyVisionPolicy = 5,
        TimeVSCostPriority = 6,
        RLAgent = 7
    };
    
    static HeadBehaviour* getInstance();

    //main function that drives choosing what to look at depending on the desired policy
    void makeVisionChoice(VisionPolicyID fieldVisionPolicy);


    void dispatchHeadJob(MobileObject *ObjectToTrack);
    
   void dispatchHeadJob(StationaryObject* ObjectToTrack);

    void dispatchHeadJob(AmbiguousObject* ObjectToTrack);
    
    //XXX: head jobs do not map from localisation space back to vision space properly, fix in here.
    /*void fieldXYToHeadingElevation(Vector2<float>* fieldposition, Vector2<float>* cameraposition) {
        
    }*/
    
    

};

#endif
