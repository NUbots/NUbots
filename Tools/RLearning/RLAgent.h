#ifndef RLAGENT_H
#define RLAGENT_H

#include "ApproximatorInterface.h"
#include "RLearningInterface.h"

#include <vector>

class RLAgent: public RLearningInterface
{
public:
    virtual void initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);

    virtual void setParameters(float alpha=0.1f, float beta=0.5, float gamma=0.9f, float lambda=0.9f,int learningIterations=1, int memory_length = 10);

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
    int memory_length;//Determines the number of observations stored when saved.

    int num_inputs;
    int num_outputs;
    int num_hidden;

    int last_action;
    vector<int> last_values;

    vector<vector<float> > values;
    vector<vector<float> > observations;
    vector<float> rewards;

    float max(vector<float> x);

};

#endif // RLAGENT_H
