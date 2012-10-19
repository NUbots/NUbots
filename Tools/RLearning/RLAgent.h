#ifndef RLAGENT_H
#define RLAGENT_H

#include "ApproximatorInterface.h"
#include "RLearningInterface.h"

#include <vector>

class RLAgent: public RLearningInterface
{
public:
    virtual void initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);

    virtual void setParameters(float alpha=0.1f, float beta=0.5, float gamma=0.9f, float lambda=0.9f,int learningIterations=1);

    virtual int getAction(vector<float> observations);//Must return integer between 0 and numberOfOutputs-1

    virtual void giveReward(float reward);

    virtual void doLearning();

    virtual void saveAgent(string agentName);

    virtual void loadAgent(string agentName);

    RLAgent();

private:
    ApproximatorInterface FunctionApproximator;
    float alpha;
    float beta;
    float gamma;
    float lambda;
    int learningIterations;

    vector<float> rewards;
    vector<vector<float> > observations;

};

#endif // RLAGENT_H
