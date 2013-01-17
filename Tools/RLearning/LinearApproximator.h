#ifndef LINEARAPPROXIMATOR_H
#define LINEARAPPROXIMATOR_H
#include <sstream>
#include <fstream>
#include <vector>

#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "ApproximatorInterface.h"
#include <cstdlib>
using namespace std;

class LinearApproximator : public ApproximatorInterface
{
public:
    LinearApproximator();

    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range = 1);

    virtual void doLearningEpisode(vector<vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1);

    virtual vector<float> getValues(vector<float> const& observations);

    virtual void saveApproximator(string agentName);

    virtual void loadApproximator(string agentName);

protected:
    vector<vector<float> > weights;

    int num_inputs;
    int num_outputs;


    vector<vector<float> > values_output;

};

#endif // LINEARAPPROXIMATOR_H
