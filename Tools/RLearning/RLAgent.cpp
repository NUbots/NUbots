#include "RLAgent.h"

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
}
/*! @brief
    Saves the agent. Calling method saves the agent to a default file. Information includes:
        -Parameters
        -Function generator
        -Lifespan
        -N most recent rewards and observations
 */
void RLAgent::saveAgent(){

}
/*! @brief
        Loads agent from file, default given as in save agent.
*/
void RLAgent::loadAgent(){

}
/*! @brief
        Sets the parameters for the learning algorithm.
*/
void RLAgent::setParameters(float alpha, float beta, float gamma, float lambda,int learningIterations){
    this->alpha = alpha;
    this->beta = beta;
    this->gamma = gamma;
    this->lambda = lambda;
    this->learningIterations = learningIterations;
}
/*! @brief
        Gives a reward to the agent. The reward is stored in a vector until learning is called.
*/
void RLAgent::giveReward(float reward){
    vector<float> value();

    rewards.push_back(reward);
}
/*! @brief
        Picks the action that will maximise reward based on the function approximator.
*/
int RLAgent::getAction(vector<float> observation){
    observations.push_back(observation);
    vector<float> Qvalues = FunctionApproximator.getValues(observation);
    int BestAction = 0;
    int BestReward = 0;
    for(int i = 0; i<Qvalues.size();i++){
        if (BestReward<Qvalues[i]){
            BestReward=Qvalues[i];
            BestAction=i;
        }
    }
    lastAction = BestAction;
    return BestAction;
}
/*! @brief
        Performs learning by modifying the function approximator to reflect rewards and observations
*/
void RLAgent::doLearning(){


    for (int i =0;i<learningIterations;i++){
        FunctionApproximator.doLearningEpisode(observations, values);
    }

}
