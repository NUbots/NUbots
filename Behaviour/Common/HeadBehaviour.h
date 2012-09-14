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


#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "NUPlatform/NUCamera/NUCameraData.h"
#include "Tools/Math/General.h"
#include "Infrastructure/NUBlackboard.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "nubotdataconfig.h"

class HeadBehaviour {

private:
    //static HeadBehaviour* Instance;
    
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
    
    //sets the tie the action will finish so a new one can be chosen
    float actionStartTime;
    float actionEndTime;
    int actionObjectID;
    
    //holds the last vision policy used
    int lastVisionPolicy;
    
    //returns the ID of the cheapest landmark to look at
    int getCheapestLandmark () {
        vector<StationaryObject> landmarks;
        landmarks.push_back(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST ]);
        landmarks.push_back(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST ]);
        landmarks.push_back(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST ]);
        landmarks.push_back(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST ]);
        vector<float> landmarkCosts;
        
        getLandmarkCosts(landmarks,&landmarkCosts);
        int minCostIndex = -1;
        float minCost = 10000000.;
        
        for (int i = 0; i < landmarkCosts.size(); i++) {
            if (landmarkCosts[i] < minCost) {
                minCostIndex = i;
                minCost = landmarkCosts[i];
            }
        }        
        
        return landmarks[minCostIndex].getID();
    }
    

    //returns euclidean cost squared of relative head movements for each set of landmarks
    void getLandmarkCosts (vector<StationaryObject> landmarks, vector<float>* landmarkCosts) {
        Vector2<float> relativeMovement;
        for (int i = 0; i < landmarks.size(); i++) {
            getRequiredHeadMovement(landmarks[i].getFieldLocation(),&relativeMovement);

            landmarkCosts->push_back(relativeMovement.x*relativeMovement.x+relativeMovement.y*relativeMovement.y);
        }
    }
    
    //returns the relative head movement required to see an object
    void getRequiredHeadMovement(Vector2<float> fieldPosition, Vector2<float>* headMovement) {
        
        //calculate relative field position
        float relativeX = fieldPosition.x-Blackboard->Objects->self.wmX();
        float relativeY = fieldPosition.y-Blackboard->Objects->self.wmY();
        
        //calculate absolute head position
        float cameraHeight;
        Blackboard->Sensors->getCameraHeight(cameraHeight);
        float absolutePitch = atan2(cameraHeight, relativeX) - 0.5*m_CAMERA_FOV_Y;
        float absoluteYaw = atan2(relativeY, relativeX);
        
        //find current camera angles
        float cameraPitch, cameraYaw;
        Blackboard->Sensors->getPosition(NUSensorsData::HeadPitch,cameraPitch);
        Blackboard->Sensors->getPosition(NUSensorsData::HeadYaw,cameraYaw);
        
        //assign relative movements
        headMovement->x = absoluteYaw-cameraYaw;
        headMovement->y = absolutePitch-cameraPitch;
        
        //do normalisation for degree checking
        if (headMovement->x > mathGeneral::PI) {
            headMovement->x = -2*mathGeneral::PI + headMovement->x;
        } else if (headMovement->x < -mathGeneral::PI) {
            headMovement->x = 2*mathGeneral::PI + headMovement->x;
        }
        
        if (headMovement->y > mathGeneral::PI) {
            headMovement->y = -2*mathGeneral::PI + headMovement->y;
        } else if (headMovement->y < -mathGeneral::PI) {
            headMovement->y = 2*mathGeneral::PI + headMovement->y;
        }
    }
    
    bool ObjectNotSeen() {
		if (actionObjectID < 0) {
			return true;
		}
        if (actionObjectID >= FieldObjects::NUM_MOBILE_FIELD_OBJECTS) {
            return actionStartTime > Blackboard->Objects->stationaryFieldObjects[actionObjectID - FieldObjects::NUM_MOBILE_FIELD_OBJECTS].TimeLastSeen();
        } else if (actionObjectID >= 0) {
            return actionStartTime > Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen();
        }
        else return false;
    }
    
    HeadBehaviour() {
        NUCameraData cameraSpecs(string(CONFIG_DIR) + "CameraSpecs.cfg");
        m_CAMERA_FOV_X = cameraSpecs.m_horizontalFov;
        m_CAMERA_FOV_Y = cameraSpecs.m_verticalFov;
        landmarkSeenFrequency = 1200;
        ballSeenFrequency = 800;
        lastVisionPolicy = -1;
        maximumSearchTime = 2500.;
        actionObjectID = -1;
        
    }
    
    void doPriorityListPolicy() {
        //prepare vision priority times
        //cout << "doing priority list policy" << flush;
        float timeSinceBallSeen = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSinceLastSeen()+ballFocusBias;
        float timeSinceLandmarkSeen = min(10000000.f,Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST ].TimeSinceLastSeen());
        timeSinceLandmarkSeen = min(timeSinceLandmarkSeen,Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST ].TimeSinceLastSeen());
        timeSinceLandmarkSeen = min(timeSinceLandmarkSeen,Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST ].TimeSinceLastSeen());
        timeSinceLandmarkSeen = min(timeSinceLandmarkSeen,Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST ].TimeSinceLastSeen());
        //cout << "Ball: " << timeSinceBallSeen << "; Landmark: " << timeSinceLandmarkSeen << endl;
        //check what we need to prioritise looking at
        if (timeSinceBallSeen/ballSeenFrequency > timeSinceLandmarkSeen/landmarkSeenFrequency) {
            dispatchHeadJob(&Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL]);
        } else {
            dispatchHeadJob(&Blackboard->Objects->stationaryFieldObjects[getCheapestLandmark()]);
        }
    }

public:
    enum VisionPolicyID{
        BallFarVisionPolicy = 0,
        BallNearVisionPolicy = 1,
        BallLostVisionPolicy = 2,
        RobotLostVisionPolicy = 3,
        BallOnlyVisionPolicy = 4,
        LandmarkOnlyVisionPolicy = 5
    };
    
    static HeadBehaviour* getInstance() {
        //if (!Instance) {
        static HeadBehaviour* Instance = new HeadBehaviour();
        //}
        return Instance;
    }

    //main function that drives choosing what to look at depending on the desired policy
    void makeVisionChoice(VisionPolicyID fieldVisionPolicy) {
        
        //If we are still moving to look at something else, don't make a new decision
        if (Blackboard->Sensors->GetTimestamp() < actionStartTime+maximumSearchTime && 
            fieldVisionPolicy == lastVisionPolicy &&
            ObjectNotSeen()) {
            //XXX: add a new head dispatch when things are standardised
			//cout << "Blocking on vision" << endl;
            return;
        }
        
		
		//if we didn't see the object, we may need to relocalise
		if (actionObjectID >= 0 and ObjectNotSeen() and fieldVisionPolicy == lastVisionPolicy) {
			Blackboard->Jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
			actionStartTime = Blackboard->Sensors->GetTimestamp()+4000;
			actionObjectID = -1;
			//cout << "objects lost, searching" << endl;
			return;
		}
		
		//cout << "looking for something now" << endl;
		lastVisionPolicy = fieldVisionPolicy;
        
        switch (fieldVisionPolicy) {
            case BallFarVisionPolicy:
                landmarkSeenFrequency = 1200;
                ballSeenFrequency = 800;
                ballFocusBias = 200.;
                doPriorityListPolicy();
                break;
            case BallNearVisionPolicy:
                landmarkSeenFrequency = 1400;
                ballSeenFrequency = 600;
                ballFocusBias = 800.;
                doPriorityListPolicy();
                break;
            case BallLostVisionPolicy:
            case RobotLostVisionPolicy:
            case BallOnlyVisionPolicy:
                landmarkSeenFrequency = 1000000.;
                ballSeenFrequency = 10;
                ballFocusBias = 1000.;
                doPriorityListPolicy();
                break;
            case LandmarkOnlyVisionPolicy:
                landmarkSeenFrequency = 10.;
                ballSeenFrequency = 1000000.;
                ballFocusBias = -1000.;
                doPriorityListPolicy();
                break;
        }   
    }

    void dispatchHeadJob(MobileObject* ObjectToTrack) {
        //initiate a new pan job using the robots estimated standard deviation of heading as the pan width
        actionObjectID = ObjectToTrack->getID();
        actionStartTime = Blackboard->Sensors->GetTimestamp();
		if (ObjectToTrack->isObjectVisible()) {
			Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
		} else {
			Blackboard->Jobs->addMotionJob(new HeadPanJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
		}
    }
    
    void dispatchHeadJob(StationaryObject* ObjectToTrack) {
        //initiate a new pan job using the robots estimated standard deviation of heading as the pan width
        actionObjectID = ObjectToTrack->getID() + FieldObjects::NUM_MOBILE_FIELD_OBJECTS;
        actionStartTime = Blackboard->Sensors->GetTimestamp();
        if (ObjectToTrack->isObjectVisible()) {
			Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
		} else {
			Blackboard->Jobs->addMotionJob(new HeadPanJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
		}
    }
    
    //XXX: head jobs do not map from localisation space back to vision space properly, fix in here.
    /*void fieldXYToHeadingElevation(Vector2<float>* fieldposition, Vector2<float>* cameraposition) {
        
    }*/
    
    

};

