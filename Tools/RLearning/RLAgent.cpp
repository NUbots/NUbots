#include "RLAgent.h"

RLAgent::RLAgent():RLearningInterface()
{

}

void RLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens){
    FunctionApproximator.initialiseApproximator(numberOfInputs,numberOfOutputs,numberOfHiddens);
}

void RLAgent::saveAgent(){

}

void RLAgent::loadAgent(){

}

void RLAgent::setParameters(float alpha, float beta, float gamma, float lambda,int learningIterations){
    this->alpha = alpha;
    this->beta = beta;
    this->gamma = gamma;
    this->lambda = lambda;
    this->learningIterations = learningIterations;
}

void RLAgent::giveReward(float reward){
    rewards.push_back(reward);
}

int RLAgent::getAction(vector<float> observations){
    return FunctionApproximator.getValues(observations);
}

void RLAgent::doLearning(){
    FunctionApproximator.doLearningEpisode(observations, rewards);
}
