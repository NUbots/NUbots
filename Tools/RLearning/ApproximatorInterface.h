


#ifndef APPROXIMATORINTERFACE_H
#define APPROXIMATORINTERFACE_H



class ApproximatorInterface {
    
    
    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);
    
    virtual void doLearningEpisode(vector<vector<float> > const& observations, vector<float> const& rewards, float stepSize=0.1, int iterations=1);
    
    virtual vector<float> getValues(vector<float> const& observations);//Should return a vector of length numberOfOutputs, corresponding to the Q-values for each action/output.
    
    virtual void saveApproximator(string agentName);
    
    virtual void loadApproximator(string agentName);
    
    
};

#endif
