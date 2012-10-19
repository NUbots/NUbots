#ifndef RLAGENT_H
#define RLAGENT_H

#include "ApproximatorInterface.h"
#include "RLearningInterface.h"

#include <vector>

class RLAgent: public RLearningInterface
{
public:

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
