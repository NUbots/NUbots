#ifndef CLUSTERCANDIDATES_H
#define CLUSTERCANDIDATES_H

#include <vector>
#include <queue>

#include "basicvisiontypes.h"
#include "VisionTools/classificationcolours.h"
#include "VisionTypes/transition.h"
#include "VisionTypes/objectcandidate.h"

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
