


#ifndef APPROXIMATORINTERFACE_H
#define APPROXIMATORINTERFACE_H



class ApproximatorInterface {
    
    
    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);
    
    virtual void doLearningEpisode(vector<vector<float>> const& observations, vector<float> const& rewards, float stepSize=0.1, int iterations=1);
    
    virtual void getValues(vector<float> const& observations);
    
    virtual void saveApproximator(string agentName);
    
    virtual void loadApproximator(string agentName);
    
    
};

#endif
