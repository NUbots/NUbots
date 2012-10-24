#include "RLAgent.h"
#include <sstream>
#include <fstream>
/*! @brief Constructor
        Sets parameters to standard values and calls constructor of function approximator.
*/
RLAgent::RLAgent():RLearningInterface()
{
   setParameters(1.0,1.0,1.0,1.0,1);
   FunctionApproximator();
}

/*! @brief  Agent initialiser
  Initialises function approximator by calling initialiseApproximator
*/
void RLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens){
    FunctionApproximator.initialiseApproximator(numberOfInputs,numberOfOutputs,numberOfHiddens);
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    num_hidden = numberOfHiddens;
    setParameters();
}

/*! @brief
    Saves the agent. Calling method saves the agent to a default file. Information includes:
        -Parameters
        -Function generator
        -Lifespan
        -N most recent rewards and observations for motivation function
 */
void RLAgent::saveAgent(string agentName){
    FunctionApproximator.saveApproximator(agentName);

    ifstream save_file;
    string file_name = agentName+"_agent";
    save_file.open(file_name,ios_base::in);


    save_file << alpha<<" ";
    save_file << beta<<" ";
    save_file << gamma<<" ";
    save_file << lambda<<" ";
    save_file << learningIterations<<" ";
    save_file << memory_length <<" ";

    save_file << num_inputs << " ";
    save_file << num_outputs<<" ";
    save_file << num_hidden <<" ";

    //Add zero values to front of observations/rewards/values to make up to size memory_length.
    vector<float> zero_observation(num_inputs.size());
    vector<float> zero_value(num_inputs.size());
    //Assuming |obs|=|values|=|rewards|
    while(memory_length-observations.size()>0){
        observations.insert(observations.begin(),zero_observation);
        rewards.insert(rewards.begin(),0);
        values.insert(values.begin(),zero_value);
    }


    //Save observations
    for(int i =0; i< memory_length;i++){
        for (int j=0;j<observations[i].size();j++){
            save_file<<observations[i][j]<<" ";
        }
        save_file <<"\n";
    }
    save_file <<"\n";

    //Save rewards
    for(int i =0; i< memory_length;i++){
        save_file<<rewards[i]<<" ";
    }
    save_file <<"\n";

    //Save values
    for(int i =0; i< memory;i++){
        for (int j=0;j<values[i].size();j++){
            save_file<<values[i][j]<<" ";
        }
        save_file <<"\n";
    }
}
/*! @brief
        Loads agent from file, default given as in save agent.
*/
void RLAgent::loadAgent(string agentName){
    FunctionApproximator.loadApproximator(agentName);
    ifstream load_file;
    string file_name = agentName+"_agent";
    load_file.open(file_name,ios_base::in);
    string tmp;

    load_file >> alpha;
    load_file >> beta;
    load_file >> gamma;
    load_file >> lambda;
    load_file >> learningIterations;
    load_file >> memory_length;

    load_file >> num_inputs;
    load_file >> num_outputs;
    load_file >> num_hidden;

    vector<vector<float> > obs(memory_length, vector<float> (num_outputs,0));
    observations = obs;

    for(int i =0; i< memory;i++){
        for (int j=0;j<observations[i].size();j++){
            load_file >> observations[i][j];
        }
    }

    vector<float> rew(memory_length, 0);
    rewards = rew;

    //Save rewards
    for(int i =0; i< memory_length;i++){
        load_file >> rewards[i];
    }


    vector<vector<float> > val(memory_length, vector<float> (num_inputs,0));
    values = val;
    //Save values
    for(int i =0; i< memory_length;i++){
        for (int j=0;j<values[i].size();j++){
            load_file>>values[i][j];
        }
    }
}

/*! @brief
        Sets the parameters for the learning algorithm.
*/
void RLAgent::setParameters(float alpha, float beta, float gamma, float lambda,int learningIterations, int memory_length){
    this->alpha = alpha;//see line 81
    this->beta = beta;
    this->gamma = gamma;//see line 81
    this->lambda = lambda;
    this->learningIterations = learningIterations;
    this->memory_length = memory_length;
}
/*! @brief
        Gives a reward to the agent. The reward is stored in a vector until learning is called.
    Should be called after getAction(observation).
*/
void RLAgent::giveReward(float reward){
    rewards.push_back(reward);

    //Add last_values to values list.
    values.push_back(last_values);
}
/*! @brief
        Picks the action that will maximise reward based on the function approximator.
*/
int RLAgent::getAction(vector<float> observation){
    observations.push_back(observation);
    last_values = FunctionApproximator.getValues(observation);
    int BestAction = 0;
    int BestReward = 0;
    for(int i = 0; i<last_values.size();i++){
        if (BestReward<last_values[i]){
            BestReward=last_values[i];
            BestAction=i;
        }
    }
    last_action = BestAction;
    return BestAction;
}
/*! @brief
        Performs learning by modifying the function approximator to reflect rewards and observations
*/
void RLAgent::doLearning(){
    for (int i =0;i<learningIterations;i++){
        /*Iterate backwards over all observations except latest observation as the next state is not known*/
        for(int observation_num=observations.size()-2; observation_num>-1;observation_num--){
            for(int action = 0; action<num_outputs; action++){
                values[observation_num][action]+= alpha*(rewards[observation_num]+gamma*max(values[observation_num+1])-values[observation_num][action]);
            }
        }
    }
    FunctionApproximator.doLearningEpisode(observations, values);
}

float RLAgent::max(vector<float> x){
    float best_value = x[0];
    for(int i=1; i<x.size();i++){
        if (best_value<x[i]) best_value=x[i];
    }
    return best_value;
}
