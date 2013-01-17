#ifndef GAUSSIANAPPROXIMATOR_H
#define GAUSSIANAPPROXIMATOR_H

class GaussianApproximator : public ApproximatorInterface
{
public:
    GaussianApproximator();

    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);

    virtual void doLearningEpisode(vector<vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1);

    virtual vector<float> getValues(vector<float> const& observations);

    virtual void saveApproximator(string agentName);

    virtual void loadApproximator(string agentName);

protected:
    vector<float> resolutions;
    vector<float> ranges;

    int num_inputs;
    int num_outputs;



};

#endif // GAUSSIANAPPROXIMATOR_H
