#ifndef FOURIERAPPROXIMATOR_H
#define FOURIERAPPROXIMATOR_H

//#include <sstream>
#include <fstream>
#include <vector>

#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "ApproximatorInterface.h"
#include "FourierFunction.h"
using namespace std;

class FourierApproximator : public ApproximatorInterface
{
public:
    FourierApproximator(bool fully_coupled, float learning_rate);

    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range=10);

    virtual void doLearningEpisode(vector< vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1);

    virtual vector<float> getValues(vector<float> const& observations);

    virtual void saveApproximator(string agentName);

    virtual void loadApproximator(string agentName);

protected:

    int num_outputs;
    int num_inputs;
    bool fully_coupled;
    float learning_rate;

    vector<FourierFunction> value_action_functions;

};

#endif // FOURIERAPPROXIMATOR_H
