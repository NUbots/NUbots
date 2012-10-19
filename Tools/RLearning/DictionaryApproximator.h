


#ifndef DICTAPPROXIMATOR_H
#define DICTAPPROXIMATOR_H

#include < map >
#include <string>
using namespace std;


class DictionaryApproximator {

private:
    int tileMultiplier,numInputs,numOutputs;
    map<string,float> approximator;
    float getValue(vector<float> const& observations,int action);
    float setValue(vector<float> const& observations,int action,float value)
    string getRepresentation(vector<float> const& observations,int action)
    
public:
    
    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);
    
    virtual void doLearningEpisode(vector< vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1);
    
    virtual vector<float> getValues(vector<float> const& observations);
    
    virtual void saveApproximator(string agentName);
    
    virtual void loadApproximator(string agentName);
    
    
};

#endif
