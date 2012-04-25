#ifndef SEGMENTFILTER_H
#define SEGMENTFILTER_H

#include <vector>
#include <fstream>

#include "VisionTypes/colourreplacementrule.h"
#include "VisionTypes/colourtransitionrule.h"
#include "VisionTypes/transition.h"
#include "VisionTypes/segmentedregion.h"

class SegmentFilter
{
public:
    SegmentFilter();
    void run() const;
        
private:
    //non mutating
    void preFilter(const SegmentedRegion& scans, SegmentedRegion &result) const;
    void filter(const SegmentedRegion& scans, map<VisionFieldObject::VFO_ID, vector<Transition> >& result) const;
    
    void checkRuleAgainstRegion(const SegmentedRegion& scans, const ColourTransitionRule& rule, vector<Transition>& matches) const;
    void applyReplacements(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after, vector<ColourSegment>& replacement, ScanDirection dir) const;
        
    void joinMatchingSegments(vector<ColourSegment>& line) const;
    //mutating
    void loadTransitionRules(string filename);
    void loadReplacementRules(string filename);
    
public:
    static const bool PREFILTER_ON = true;
    
private:
    vector<ColourReplacementRule> replacement_rules_h;
    vector<ColourReplacementRule> replacement_rules_v;
    vector<ColourTransitionRule> rules_h;
    vector<ColourTransitionRule> rules_v;
    
};

#endif // SEGMENTFILTER_H
