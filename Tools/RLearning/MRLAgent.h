#ifndef MRLAGENT_H
#define MRLAGENT_H

#include "ApproximatorInterface.h"
#include "RLearningInterface.h"
#include "RLAgent.h"

#include <vector>

class MRLAgent: public RLAgent
{
public:
    float giveMotivationReward();
    float wundtFunction();

};

#endif // MRLAGENT_H
