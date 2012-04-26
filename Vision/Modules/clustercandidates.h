#ifndef CLUSTERCANDIDATES_H
#define CLUSTERCANDIDATES_H

#include <vector>
#include <queue>

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/transition.h"
#include "Vision/VisionTypes/objectcandidate.h"

using namespace std;

class ClusterCandidates
{
public:
    ClusterCandidates();
    
//    vector<ObjectCandidate> classifyCandidatesPrims(vector<Transition> &transitions,
//                                                    const vector<PointType >&fieldBorders,
//                                                    const vector<ClassIndex::Colour> &validColours,
//                                                    int spacing,
//                                                    float min_aspect, float max_aspect, int min_transitions);
};

#endif // CLUSTERCANDIDATES_H
