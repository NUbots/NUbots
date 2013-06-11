/*! @file HeadBehaviour.cpp
    @brief Head behaviour implementation for better head behaviour control. Chooses what landmark to look at next.

    @author Jake Fountain and Josiah Walker

 Copyright (c) 2012

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



#include <vector>
#include <cmath>

/*! @brief Checks if the last object looked at was seen. An object is counted as seen if it is seen for at least CONFIRMATION_TIME.
  */

bool HeadBehaviour::ObjectNotSeen() {
    if (actionObjectID < 0) {
        return true;
    }
    if (actionObjectID >= FieldObjects::NUM_MOBILE_FIELD_OBJECTS) {
        return ((actionStartTime > Blackboard->Objects->stationaryFieldObjects[actionObjectID - FieldObjects::NUM_MOBILE_FIELD_OBJECTS].TimeLastSeen())
                and Blackboard->Objects->stationaryFieldObjects[actionObjectID - FieldObjects::NUM_MOBILE_FIELD_OBJECTS].TimeSeen()<CONFIRMATION_TIME);
    } else if (actionObjectID >= 0) {
        return (actionStartTime > Blackboard->Objects->mobileFieldObjects[actionObjectID].TimeLastSeen() and
                Blackboard->Objects->mobileFieldObjects[actionObjectID].TimeSeen() < CONFIRMATION_TIME);
    }
    else return false;
}



/*! @brief Constructor initialises all parameters and initialises head behaviour (motivated) reinforcement learning agent.
  */
HeadBehaviour::HeadBehaviour():Mrlagent(){
    MAX_PERCEPT_RANGESIZE = 10;
    NUCameraData cameraSpecs(string(CONFIG_DIR) + "CameraSpecs.cfg");
    m_CAMERA_FOV_X = cameraSpecs.m_horizontalFov;
    m_CAMERA_FOV_Y = cameraSpecs.m_verticalFov;
    head_logic = HeadLogic::getInstance();
    landmarkSeenFrequency = 1200;
    ballSeenFrequency = 800;
    lastVisionPolicy = -1;
    maximumSearchTime = 1000.;
    CONFIRMATION_TIME = 250.;
    buttonPressTime = 0.;
    actionObjectID = -1;
    time_since_last_localisation = 0;
    ACTIONS_PER_STATE = 20;
    actions_taken_this_state = 0;

    rewards_log_pathname = "nubot/Config/Darwin/HeadRewards.csv";//Rewards are not recorded unless uncommented in calculateReward() method
    agent_filename = "Order30_CombinedAgent";
    give_rew_to_mot = true;


    //Policy Parameters:
    prioritise_localisation_policy_bias = 0.5;//Probability prioritising localisation will give no looking at mobile objects
    prioritise_ball_policy_bias = 0.5; //Probability prioritising localisation will give no looking at stationary objects
    srand(Blackboard->Sensors->GetTimestamp());

    //MRL Agent
    //Get dummy observation to determine input size to MRLagent
    vector<float> inputs = getPercept();
    try{
        Mrlagent.loadMRLAgent(agent_filename);
        cout<<"HeadBehaviour::HeadBehaviour():Mrlagent() - agent loaded successfully."<<endl;
    } catch (string s){
        Mrlagent.log("\nHeadBehaviour::HeadBehaviour():Mrlagent() - Agent did not load properly. Resetting..."+s);
        cout<<s<<"\nHeadBehaviour::HeadBehaviour():Mrlagent() - Agent did not load properly. Resetting..."<<endl;
        try{
            Mrlagent.initialiseAgent(inputs.size()
                                     //Calculate output size = sum of the number of stationary, mobile and ambiguous objects.
                                     ,head_logic->relevantObjects[0].size()+head_logic->relevantObjects[1].size()+head_logic->relevantObjects[2].size()
                                     /*Order of fourier approx.*/
                                     ,30
                                     //max_percept_values
                                     ,MAX_PERCEPT_RANGESIZE);
            Mrlagent.setParameters(0.5,0,0.1,0.5,5,15, false);//Default for Combined or motivated agent.

        }catch(string s){
           cout<<s<<endl;
           throw string("");
        }
    }

    //Log training session date and time.
    ofstream save_file;
    save_file.open(rewards_log_pathname.c_str(),ofstream::app);
    time_t t = time(0);// get time now
    save_file<<"\n";
    save_file << ctime(&t);
    save_file << " "<< agent_filename << " ";
    save_file.close();
}

/*! @brief Destructor
  */
HeadBehaviour::~HeadBehaviour(){
    Mrlagent.saveMRLAgent("HeadBehaviourMRL");
    delete head_logic;
}

/*! @brief Old, broken priority list policy. Better implemented by doTimeVSCostPriorityPolicy.
  */
void HeadBehaviour::doPriorityListPolicy() {

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
        dispatchHeadJob(&Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL]);
    } else {
        int ran = rand()%Blackboard->Objects->stationaryFieldObjects.size();
        dispatchHeadJob(&(Blackboard->Objects->stationaryFieldObjects[ran]));
    }
}
/*! @brief Policy chooses lowest cost object not seen for longest time
*/
void HeadBehaviour::doTimeVSCostPriorityPolicy(){
    //Do highest priority of HeadLogic's interesting objects. Priority = (time since last seen)/cost

    vector<float> times = head_logic->getTimeSinceLastSeenSummary();
    vector<float> costs = head_logic->getCostList((float)1.0,(float)1.0);//Look at whole screen.


    //Debug:
    cout<<" HeadBehaviour::doTimeVSCostPriorityPolicy(): cost list = [";
    for(int i = 0; i<costs.size(); i++){
        cout<<" "<<costs[i]<<",";
    }
    cout<<"]"<<endl;
    cout<<" HeadBehaviour::doTimeVSCostPriorityPolicy(): time since last seen list = [";
    for(int i = 0; i<times.size(); i++){
        cout<<" "<<times[i]<<",";
    }
    cout<<"]"<<endl;

    vector<float> priorities(0);
    if(times.size()==costs.size()){
        for (int i = 0; i<times.size(); i++ ){
            if(costs[i]==0.0){
                priorities.push_back((float)0);//Don't look at something already in vision.
            } else {
                priorities.push_back((float)times[i]/costs[i]);
            }
        }
    }else cout<<"HeadBehaviour::doTimeVSCostPriorityPolicy(): Times and costs different sizes()."<<endl;

    cout<<" HeadBehaviour::doTimeVSCostPriorityPolicy(): priorities = [";
    for(int i = 0; i<priorities.size(); i++){
        cout<<" "<<priorities[i]<<",";
    }
    cout<<"]"<<endl;

    float bestPriority = 0;
    int bestObject = 0;
    //Find index of best priority Object
    for (int i = 0; i< priorities.size();i++){
        if (bestPriority<priorities[i]){
            bestPriority = priorities[i];
            bestObject = i;
        }
    }

    int ob_type = head_logic->getObjectType(bestObject);

    if (ob_type == 1){
        dispatchHeadJob((MobileObject*)head_logic->getObject(bestObject));
        cout<<" HeadBehaviour::doPriorityListPolicy() Performing priority policy - Mobile Object: "<< bestObject <<endl;
    } else if (ob_type == 2){
        dispatchHeadJob((AmbiguousObject*)head_logic->getObject(bestObject));
        cout<<" HeadBehaviour::doPriorityListPolicy() Performing priority policy - Ambiguous Object: "<< bestObject <<endl;
    } else {
        dispatchHeadJob((StationaryObject*)head_logic->getObject(bestObject));
        cout<<" HeadBehaviour::doPriorityListPolicy() Performing priority policy - Stationary Object: "<< bestObject <<endl;
    }


    //Warning:temporary p
}



/*! @brief Returns head behaviour pointer
*/

HeadBehaviour* HeadBehaviour::getInstance() {
    //if (!Instance) {
    static HeadBehaviour* Instance = new HeadBehaviour();
    //}
    return Instance;
}

/*! @brief Main method for making head decisions. Agent policies save the changes every ACTIONS_PER_STATE number of actions.
*/
//main function that drives choosing what to look at depending on the desired policy
void HeadBehaviour::makeVisionChoice(VisionPolicyID fieldVisionPolicy) {

    float current_time = Blackboard->Sensors->GetTimestamp();
    //If we are still moving to look at something else, don't make a new decision
    if (
        current_time < actionStartTime+maximumSearchTime &&
        fieldVisionPolicy == lastVisionPolicy and
        ObjectNotSeen()
       )
    {
        return;
    }
//Localisation hack: activated to improve performance if robot is getting lost too much.
    //if we didn't see the object, we may need to relocalise
  /*  if (fieldVisionPolicy!=TimeVSCostPriority and actionObjectID >= 0 and ObjectNotSeen() and fieldVisionPolicy == lastVisionPolicy and last_reward<-0.9){
        Blackboard->Jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
        actionStartTime = Blackboard->Sensors->GetTimestamp()+4000;
        actionObjectID = -1;
        time_since_last_localisation = current_time;

        cout << "objects lost, searching" << endl;
        return;
    }*/



    lastVisionPolicy = fieldVisionPolicy;

    switch (fieldVisionPolicy) {
        case PrioritiseLocalisationPolicy:
            {
                vector<int> object_selection_vector = head_logic->getValidObjectsToLookAt();
                if(rand()< prioritise_localisation_policy_bias*RAND_MAX){
                    for(int i =0; i<head_logic->relevantObjects[HeadLogic::MOBILE_OBJECT].size(); i++){
                        object_selection_vector[head_logic->getObjectIndex(HeadLogic::MOBILE_OBJECT,i)] = 0;//For each mobile object set selection possibility to zero sometimes.
                    }
                    for(int i =0; i<head_logic->relevantObjects[HeadLogic::AMBIGUOUS_OBJECT].size(); i++){
                        object_selection_vector[head_logic->getObjectIndex(HeadLogic::AMBIGUOUS_OBJECT,i)] = 0;//For each ambiguous object set selection possibility to zero sometimes.
                    }
                }
                doAgentBasedPolicy(object_selection_vector);//TODO:add bias to landmarks
                break;
            }
        case PrioritiseBallPolicy:
            {
                vector<int> object_selection_vector = head_logic->getValidObjectsToLookAt();
                if(rand()< prioritise_ball_policy_bias*RAND_MAX){
                    for(int i =0; i<head_logic->relevantObjects[HeadLogic::STATIONARY_OBJECT].size(); i++){
                        object_selection_vector[head_logic->getObjectIndex(HeadLogic::STATIONARY_OBJECT,i)] = 0;//For each stationary object set selection possibility to zero sometimes.
                    }
                    for(int i =0; i<head_logic->relevantObjects[HeadLogic::AMBIGUOUS_OBJECT].size(); i++){
                        object_selection_vector[head_logic->getObjectIndex(HeadLogic::AMBIGUOUS_OBJECT,i)] = 0;//For each ambiguous object set selection possibility to zero sometimes.
                    }
                }
                doAgentBasedPolicy(object_selection_vector);//TODO:add bias to ball
                break;
            }
        case LookAtBallPolicy:
            {
                vector<int> object_selection_vector = head_logic->getValidObjectsToLookAt();
                //Look at nothing but the ball:
                for (int i =0; i<object_selection_vector.size();i++){
                    object_selection_vector[i]=0;
                }
                object_selection_vector[head_logic->getObjectIndex(HeadLogic::MOBILE_OBJECT,FieldObjects::FO_BALL)] = 1;//Set ball to be looked at
                doAgentBasedPolicy(object_selection_vector);//TODO:add bias to ball
                break;
            }
        case LookForBallPolicy://TODO: implement ball localisation head movement
            {
                vector<int> object_selection_vector = head_logic->getValidObjectsToLookAt();
                //Look at nothing but the ball:
                for (int i =0; i<object_selection_vector.size();i++){
                    object_selection_vector[i]=0;
                }
                object_selection_vector[head_logic->getObjectIndex(HeadLogic::MOBILE_OBJECT,FieldObjects::FO_BALL)] = 1;//Set ball to be looked at
                doAgentBasedPolicy(object_selection_vector);
                break;
            }
        case LookForFieldObjectsPolicy:
            {
            vector<int> object_selection_vector = head_logic->getValidObjectsToLookAt();
            for(int i =0; i<head_logic->relevantObjects[HeadLogic::MOBILE_OBJECT].size(); i++){
                object_selection_vector[head_logic->getObjectIndex(HeadLogic::MOBILE_OBJECT,i)] = 0;//For each mobile object set selection possibility to zero always.
            }
            for(int i =0; i<head_logic->relevantObjects[HeadLogic::AMBIGUOUS_OBJECT].size(); i++){
                object_selection_vector[head_logic->getObjectIndex(HeadLogic::AMBIGUOUS_OBJECT,i)] = 0;//For each ambiguous object set selection possibility to zero always.
            }

            doAgentBasedPolicy(object_selection_vector);
            break;
            }
        case TimeVSCostPriority:
            doTimeVSCostPriorityPolicy();
            break;
        //Reinforcement learning policy
        case RLAgentPolicy:
            if (actions_taken_this_state < ACTIONS_PER_STATE){
                 doRLAgentPolicy();
                 actions_taken_this_state++;
            }else{
                Mrlagent.saveMRLAgent(agent_filename);
                actions_taken_this_state = 0;
            }
            break;
        //Motivated reinforcement
        case MRLAgentPolicy:
            if (actions_taken_this_state < ACTIONS_PER_STATE){
                 doMRLAgentPolicy();
                 actions_taken_this_state++;
            }else{
                Mrlagent.saveMRLAgent(agent_filename);
                actions_taken_this_state = 0;
            }
            break;

        //Training: do ACTIONS_PER_STATE actions and wait for middle button press.
        case RLAgentTrainingPolicy:
            if (actions_taken_this_state < ACTIONS_PER_STATE){
                 doRLAgentPolicy();//doRLAgentPolicy();
                 actions_taken_this_state++;
            }else{
                //Wait for button press
                Mrlagent.saveMRLAgent(agent_filename);
                cout<<"HeadBehaviour::makeVisionChoice - RLAgentTrainingPolicy - Waiting for left button press before continuing... Move robot to new location."<<endl;
                actions_taken_this_state = 0;
                float button_state;
                Blackboard->Sensors->getButton(NUSensorsData::MainButton,button_state);
                while(button_state<=0){
                     Blackboard->Sensors->getButton(NUSensorsData::MainButton,button_state);
                }
            }
            break;

        //Training: do ACTIONS_PER_STATE actions and wait for middle button press.
        case MRLAgentTrainingPolicy:
            if (actions_taken_this_state < ACTIONS_PER_STATE){
                 doMRLAgentPolicy();
                 actions_taken_this_state++;
            }else{
                actions_taken_this_state = 0;
                Mrlagent.saveMRLAgent(agent_filename);
                cout<<"HeadBehaviour::makeVisionChoice - MRLAgentTrainingPolicy - Waiting for left button press before continuing... Move robot to new location."<<endl;
                //Wait for button press
                float button_state;
                Blackboard->Sensors->getButton(NUSensorsData::MainButton,button_state);
                while(button_state<=0){
                     Blackboard->Sensors->getButton(NUSensorsData::MainButton,button_state);
                }
            }
            break;

        // Simply checks the policy of the RLAgent.
        case CheckAgentPolicy:
            doCheckAgentPolicy();
            break;
    }

}

void  HeadBehaviour::doAgentBasedPolicy(vector<int> object_selection_vector){
//        cout<<"HeadBehaviour::doAgentBasedPolicy - object_selection_vector =";
//        for (int i =0; i< object_selection_vector.size();i++){
//            cout <<", "<< object_selection_vector[i]<<" ";
//        }
//        cout<< "]";
        vector<float> inputs = getPercept();
        int action = Mrlagent.getActionAndLearn(inputs,object_selection_vector,(give_rew_to_mot ? calculateReward()-0.5:0));
        float temp = (!give_rew_to_mot ? calculateReward():0);//Calculate reward to measure and record performance if not done already.

        //Send Job:
        if (action < head_logic->relevantObjects[0].size()){    //i.e. Stationary Object
            dispatchHeadJob((StationaryObject*)(head_logic->getObject(action)));
        } else if (action < head_logic->relevantObjects[1].size()+head_logic->relevantObjects[0].size()){ //i.e. Mobile Object
            dispatchHeadJob((MobileObject*)(head_logic->getObject(action)));
        } else {
            dispatchHeadJob((AmbiguousObject*)(head_logic->getObject(action)));
        }

        actions_taken_this_state++;
        if (actions_taken_this_state >= ACTIONS_PER_STATE){
            Mrlagent.saveMRLAgent(agent_filename);
            actions_taken_this_state = 0;
        }
}


/*! @brief Motivated rlagent online learning policy.
*/
void HeadBehaviour::doMRLAgentPolicy(){

    vector<float> inputs = getPercept();
    int action = Mrlagent.getActionAndLearn(inputs,head_logic->getValidObjectsToLookAt(),(give_rew_to_mot ? calculateReward()-0.5:0));
    float temp = (!give_rew_to_mot ? calculateReward():0);//Calculate reward to measure and record performance if not done already.


    if (action < head_logic->relevantObjects[0].size()){    //i.e. Stationary Object
        dispatchHeadJob((StationaryObject*)(head_logic->getObject(action)));

        //Debug text for use when training:
        // cout<<" HeadBehaviour::doMRLAgentPolicy() Performing priority policy - Stationary Object: "<< action << "- Percept = [";
        /*for(int i = 0; i<inputs.size(); i++){
            cout<<" "<<inputs[i]<<",";
        }*/
       // cout<<"]"<<endl;
    } else if (action < head_logic->relevantObjects[1].size()+head_logic->relevantObjects[0].size()){ //i.e. Mobile Object
        dispatchHeadJob((MobileObject*)(head_logic->getObject(action)));

        //Debug text for use when training:
        //cout<<" HeadBehaviour::doMRLAgentPolicy() Performing priority policy - Mobile Object: "<< action << "- Percept = [";
        /*for(int i = 0; i<inputs.size(); i++){
            cout<<" "<<inputs[i]<<",";
        }*/
        //cout<<"]"<<endl;
    } else {
        dispatchHeadJob((AmbiguousObject*)(head_logic->getObject(action)));

        //Debug text for use when training:
        /*cout<<" HeadBehaviour::doMRLAgentPolicy() Performing priority policy - Ambiguous Object: "<< action << "- Percept = [";
        /*for(int i = 0; i<inputs.size(); i++){
            cout<<" "<<inputs[i]<<",";
        }
        cout<<"]"<<endl;*/
    }

}

/*! @brief Reinforcement agent online learning policy.
*/
void HeadBehaviour::doRLAgentPolicy(){

    vector<float> inputs = getPercept();
    int action = Mrlagent.getAction(inputs,head_logic->getValidObjectsToLookAt());
    float rew = calculateReward();
    cout<< "Reward = "<<rew<<endl;
    Mrlagent.giveReward(rew);
    Mrlagent.doLearning();

    if (action < head_logic->relevantObjects[0].size()){    //i.e. Stationary Object
        dispatchHeadJob((StationaryObject*)(head_logic->getObject(action)));
        /*cout<<" HeadBehaviour::doRLAgentPolicy() Performing priority policy - Stationary Object: "<< action <<endl;
        cout<< "- Percept = [";
        for(int i = 0; i<inputs.size(); i++){
            cout<<" "<<inputs[i]<<",";
        }
        cout<<"]"<<endl;*/
    } else if (action < head_logic->relevantObjects[1].size()+head_logic->relevantObjects[0].size()){ //i.e. Mobile Object
        dispatchHeadJob((MobileObject*)(head_logic->getObject(action)));
        /*cout<<" HeadBehaviour::doRLAgentPolicy() Performing priority policy - Mobile Object: "<< action << endl;
        cout<< "Percept = [";
        for(int i = 0; i<inputs.size(); i++){
            cout<<" "<<inputs[i]<<",";
        }
        cout<<"]"<<endl;*/
    } else {
        dispatchHeadJob((AmbiguousObject*)(head_logic->getObject(action)));
        /*cout<<" HeadBehaviour::doRLAgentPolicy() Performing priority policy - Ambiguous Object: "<< action <<endl;
        cout<<  "- Percept = [";
        for(int i = 0; i<inputs.size(); i++){
            cout<<" "<<inputs[i]<<",";
        }
        cout<<"]"<<endl;*/
    }

}

/*! @brief Non-learning agent policy
*/
void HeadBehaviour::doCheckAgentPolicy()
{
    vector<float> inputs = getPercept();
    int action = Mrlagent.checkAction(inputs,head_logic->getValidObjectsToLookAt());
    calculateReward();
    if (action < head_logic->relevantObjects[0].size()){    //i.e. Stationary Object
        dispatchHeadJob((StationaryObject*)(head_logic->getObject(action)));
    } else if (action < head_logic->relevantObjects[1].size()+head_logic->relevantObjects[0].size()){ //i.e. Mobile Object
        dispatchHeadJob((MobileObject*)(head_logic->getObject(action)));
    } else {
        dispatchHeadJob((AmbiguousObject*)(head_logic->getObject(action)));
    }
}

/*! @brief Calculates reward based on localisation error of ball and self
*/
float HeadBehaviour::calculateReward(){

    float errx = Blackboard->Objects->self.sdX();
    float erry = Blackboard->Objects->self.sdY();
    float errh = Blackboard->Objects->self.sdHeading();

    Vector2<float> berr = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].getEstimatedFieldLocationError();

    float ball_reward = -(1-exp(-(berr.x*berr.x+berr.y*berr.y)/30000));
    float self_reward = -(1-exp(-(errx*errx+erry*erry+errh*errh)/800));

    //Weight in terms of distance to ball: ball close to robot means ball more important.
    float weight = 0.25+0.5*(1-exp(-fabs(head_logic->calculateMobilePolarObjectLocation(&(Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL]))[0])/250));
    float reward = 1+((1-weight)*ball_reward+weight*self_reward);

    //When reward recording desired, uncomment: Rewards are stored in rewards_log_pathname
    //recordReward(reward);
    last_reward = reward;

    return reward;
}
/*! @brief Records rewards for benchmarking purposes.
*/
void HeadBehaviour::recordReward(float r)
{
    ofstream save_file;
    save_file.open(rewards_log_pathname.c_str(),ofstream::app);
    if (actions_taken_this_state == ACTIONS_PER_STATE-1 or lastVisionPolicy==RLAgentPolicy or lastVisionPolicy==MRLAgentPolicy)
        save_file<<r<<"\n";
    else
        save_file<<r<<" ";
    save_file.close();

}

/*! @brief Current percept in order:
    -Priorities for each object
    -goal distances
    -ball distance
    -ball error
    -self error
    total size = 10 for goals and ball as important objects in head logic

    All inputs scales using sigmoids or half sigmoids to be in a range of size MAX_PERCEPT_RANGESIZE
*/

vector<float> HeadBehaviour::getPercept(){

    vector<float> inputs= head_logic->getTimeSinceLastSeenSummary();
    vector<float> costs = head_logic->getCostList(0,0);
    for(int i = 0;i<inputs.size();i++){
        inputs[i]=inputs[i]*(1+exp(-(costs[i]-8)/4));//Divide times by sigmoided costs to get priorities.
    }

    for (int i = 0; i<inputs.size(); i++){
        inputs[i] = MAX_PERCEPT_RANGESIZE*(1-exp(-inputs[i]/200.));
    }

    float lblue_goal_loc = head_logic->calculateStationaryPolarObjectLocation(&(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST]))[0];
    float rblue_goal_loc = head_logic->calculateStationaryPolarObjectLocation(&(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST]))[0];
    float lyellow_goal_loc = head_logic->calculateStationaryPolarObjectLocation(&(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST]))[0];
    float ryellow_goal_loc = head_logic->calculateStationaryPolarObjectLocation(&(Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST]))[0];

    float bgoal = (lblue_goal_loc+rblue_goal_loc)/2;
    float ygoal = (lyellow_goal_loc+ryellow_goal_loc)/2;
    inputs.push_back((MAX_PERCEPT_RANGESIZE*(1-exp(-bgoal/400.))));
    inputs.push_back((MAX_PERCEPT_RANGESIZE*(1-exp(-ygoal/400.))));
    
    vector<float> ball_location = head_logic->calculateMobilePolarObjectLocation(&(Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL]));
    ball_location[0] =(MAX_PERCEPT_RANGESIZE*(1-exp(-ball_location[0]/400.)));
    ball_location[1]*=ball_location[1];

    inputs.push_back(ball_location[0]);

    float errx = Blackboard->Objects->self.sdX();
    float erry = Blackboard->Objects->self.sdY();
    float errh = Blackboard->Objects->self.sdHeading();

    Vector2<float> berr = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].getEstimatedFieldLocationError();
    float ball_err = (MAX_PERCEPT_RANGESIZE*(1-exp(-(berr.x*berr.x+berr.y*berr.y)/30000.)));
    float self_err= (MAX_PERCEPT_RANGESIZE*(1-exp(-(errx*errx+erry*erry+errh*errh)/800.)));
    inputs.push_back(self_err);
    inputs.push_back(ball_err);
    return inputs;
}




/*! @brief Sets head behaviour to prioritise landmarks(==field objects).
    Uses agent decision making.
*/
void HeadBehaviour::prioritiseLocalisation(){
    current_policy = PrioritiseLocalisationPolicy;
}

/*! @brief  Sets head behaviour to prioritise ball, but not to the exclusion of all else.
  Uses agent decision making.

*/
void HeadBehaviour::prioritiseBall(){
    current_policy = PrioritiseBallPolicy;
}

/*! @brief  Sets head behaviour to track the ball when it is in view.
  WARNING: Currently identical to lookForBall() - both simply assign head job and track if possible; otherwise pan
*/
void HeadBehaviour::lookAtBall(){
    current_policy = LookAtBallPolicy;
}


/*! @brief Sets head behaviour to look for the ball. Used when the ball is out of vision range.
    Simply give head pan job for ball.
    WARNING: Currently identical to lookAtBall() - both simply assign head job and track if possible; otherwise pan
*/
void HeadBehaviour::lookForBall(){
    current_policy = LookForBallPolicy;
}


/*! @brief Sets the head behaviour to look for landmarks(==field objects). That is, the robot is lost; localisation information is required.
    Should not be called unless robot is lost.
    Uses agent decision making, but cuts off all mobile objects.
*/
void HeadBehaviour::lookForFieldObjects(){
    current_policy = LookForFieldObjectsPolicy;
}


/*! @brief Performs post processing and adds a head job to the joblist
*/
void HeadBehaviour::update(){
    makeVisionChoice(current_policy);
}



void HeadBehaviour::dispatchHeadJob(StationaryObject* ObjectToTrack) {
    //initiate a new pan job using the robots estimated standard deviation of heading as the pan width

    actionObjectID = ObjectToTrack->getID();
    actionStartTime = Blackboard->Sensors->GetTimestamp();
    if (ObjectToTrack->isObjectVisible()) {
        Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack));
    } else {
        Blackboard->Jobs->addMotionJob(new HeadPanJob(*ObjectToTrack));
    }
}
void HeadBehaviour::dispatchHeadJob(MobileObject* ObjectToTrack) {
    //initiate a new pan job using the robots estimated standard deviation of heading as the pan width

    actionObjectID = ObjectToTrack->getID() + FieldObjects::NUM_MOBILE_FIELD_OBJECTS;
    actionStartTime = Blackboard->Sensors->GetTimestamp();
    if (ObjectToTrack->isObjectVisible()) {
        Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack));
    } else {
        Blackboard->Jobs->addMotionJob(new HeadPanJob(*ObjectToTrack));
    }
}
void HeadBehaviour::dispatchHeadJob(AmbiguousObject* ObjectToTrack) {
    //initiate a new pan job using the robots estimated standard deviation of heading as the pan width

    actionObjectID = ObjectToTrack->getID() + FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS;
    actionStartTime = Blackboard->Sensors->GetTimestamp();
    if (ObjectToTrack->isObjectVisible()) {
        Blackboard->Jobs->addMotionJob(new HeadTrackJob(*ObjectToTrack));
    }

}





