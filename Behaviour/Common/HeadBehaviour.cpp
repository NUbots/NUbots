/*! @file HeadBehaviour.cpp
    @brief Head behaviour implementation for simpler behaviour control.

    @author J

 Copyright (c) 2012 J

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
#include "HeadBehaviour.h"

#include "nubotdataconfig.h"

#include <vector>
#include <cmath>


	    bool HeadBehaviour::ObjectNotSeen() {
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

	    HeadBehaviour::HeadBehaviour() {
	        NUCameraData cameraSpecs(string(CONFIG_DIR) + "CameraSpecs.cfg");
	        m_CAMERA_FOV_X = cameraSpecs.m_horizontalFov;
	        m_CAMERA_FOV_Y = cameraSpecs.m_verticalFov;
            head_logic = HeadLogic::getInstance();
	        landmarkSeenFrequency = 1200;
	        ballSeenFrequency = 800;
	        lastVisionPolicy = -1;
	        maximumSearchTime = 2500.;
	        actionObjectID = -1;

            vector<float> inputs= head_logic->getTimeSinceLastSeenSummary();
            vector<float> other_inputs = head_logic->getCostList(1,1);

            Mrlagent();
            Mrlagent.initialiseAgent(inputs.size()+other_inputs.size(),head_logic->relevantObjects[0].size()+head_logic->relevantObjects[1].size()+head_logic->relevantObjects[2].size(),10);
	    }

	    void HeadBehaviour::doPriorityListPolicy() {
        /*
            //Josiah's simple priority list head behaviour.
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
                dispatchHeadJob((Object)&Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL]);
                actionObjectID = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].getID() + FieldObjects::NUM_MOBILE_FIELD_OBJECTS;
	        } else {
               // dispatchHeadJob(&(Blackboard->Objects->stationaryFieldObjects[FieldObjects::]);
            }*/

	    }

        void HeadBehaviour::doTimeVSCostPriorityPolicy(){
            //Do longest seen of HeadLogic's interesting objects.

            vector<float> times = head_logic->getTimeSinceLastSeenSummary();
            vector<float> costs = head_logic->getCostList((float)1.0,(float)1.0);//Look at whole screen.
            vector<float> priorities(0);
            if(times.size()==costs.size()){
                for (int i = 0; i<times.size(); i++ ){
                    if(costs[i]==(float)0){
                        priorities.push_back((float)0);//Don't look at something already in vision.
                    } else {
                        priorities.push_back((float)times[i]/costs[i]);
                    }
                }
            }


            float bestPriority;
            int bestObject;
            //Find index of best priority Object
            for (int i = 0; i< priorities.size();i++){
                if (bestPriority>priorities[i]){
                    bestPriority = priorities[i];
                    bestObject = i;
                }
            }

            int ob_type = head_logic->getObjectType(bestObject);

            if (ob_type == 1){
                dispatchHeadJob((MobileObject*)head_logic->getObject(bestObject));
            } else if (ob_type == 2){
                dispatchHeadJob((AmbiguousObject*)head_logic->getObject(bestObject));
            } else {
                dispatchHeadJob((StationaryObject*)head_logic->getObject(bestObject));
            }


            //Warning:temporary p
        }





        HeadBehaviour* HeadBehaviour::getInstance() {
	        //if (!Instance) {
	        static HeadBehaviour* Instance = new HeadBehaviour();
	        //}
	        return Instance;
	    }

	    //main function that drives choosing what to look at depending on the desired policy
	    void HeadBehaviour::makeVisionChoice(VisionPolicyID fieldVisionPolicy) {

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
                case TimeVSCostPriority:
                    doTimeVSCostPriorityPolicy();
                case RLAgent:
                    doRLagentPolicy();
                    break;
                    //Get instructions from RL agent.
	        }
	    }


        void HeadBehaviour::doRLAgentPolicy(){
            //If the input vectors are changed, the RLagent input size must be changed.
            vector<float> inputs= head_logic->getTimeSinceLastSeenSummary();
            vector<float> other_inputs = head_logic->getCostList(1,1);

            //Combines inputs to one vector to feed to RLagent
            inputs.insert(inputs.end(), other_inputs.begin(), other_inputs.end());


            int action = Mrlagent.getAction(inputs);//Should output integer from 0 to outputs-1
            Mrlagent.giveMotivationReward();

            if (action < head_logic->relevantObjects[0].size()/*i.e. Stationary Object*/){
                dispatchHeadJob((StationaryObject*)head_logic->getObject(action));
            } else if (action < head_logic->relevantObjects[1].size()/*i.e. Mobile Object*/){
                dispatchHeadJob((MobileObject*)head_logic->getObject(action));
            } else {
                dispatchHeadJob((AmbiguousObject*)head_logic->getObject(action));
            }

        }

        void HeadBehaviour::dispatchHeadJob(StationaryObject* ObjectToTrack) {
            //initiate a new pan job using the robots estimated standard deviation of heading as the pan width

            actionObjectID = ObjectToTrack->getID();
            actionStartTime = Blackboard->Sensors->GetTimestamp();
            if (ObjectToTrack->isObjectVisible()) {
                Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
            } else {
                Blackboard->Jobs->addMotionJob(new HeadPanJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
            }
        }
        void HeadBehaviour::dispatchHeadJob(MobileObject* ObjectToTrack) {
	        //initiate a new pan job using the robots estimated standard deviation of heading as the pan width
	        actionObjectID = ObjectToTrack->getID() + FieldObjects::NUM_MOBILE_FIELD_OBJECTS;
	        actionStartTime = Blackboard->Sensors->GetTimestamp();
	        if (ObjectToTrack->isObjectVisible()) {
				Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
			} else {
				Blackboard->Jobs->addMotionJob(new HeadPanJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
            }
	    }
        void HeadBehaviour::dispatchHeadJob(AmbiguousObject* ObjectToTrack) {

            //initiate a new pan job using the robots estimated standard deviation of heading as the pan width
            actionObjectID = ObjectToTrack->getID() + FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS;
            actionStartTime = Blackboard->Sensors->GetTimestamp();
            /*if (ObjectToTrack->isObjectVisible()) {
                Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
            } else {
                Blackboard->Jobs->addMotionJob(new HeadPanJob(*ObjectToTrack, Blackboard->Objects->self.sdHeading()));
            }*/

        }

	    //XXX: head jobs do not map from localisation space back to vision space properly, fix in here.
	    /*void fieldXYToHeadingElevation(Vector2<float>* fieldposition, Vector2<float>* cameraposition) {

	    }*/



