#ifndef SEGMENTFILTER_H
#define SEGMENTFILTER_H

#include <vector>
#include <fstream>

#include "Vision/VisionTypes/colourreplacementrule.h"
#include "Vision/VisionTypes/colourtransitionrule.h"
#include "Vision/VisionTypes/segmentedregion.h"

class SegmentFilter
{
public:
    SegmentFilter();
    /**
      @brief runs the segment filter over the horizontal and vertical segments std::lists.
      This matches pairs of segments to preloaded transition rules and stores matching results
      as transitions back on the blackboard. This method also calls some smoothing prefilters on the std::lists
      which are also set by preloaded rules.
      */
    void run() const;
        
private:

    /**
      @brief runs the segment prefilter rules over a segment std::list.
      @param scans the std::lists of segments.
      @param result a smoothed result.
      */
    void preFilter(const SegmentedRegion& scans, SegmentedRegion &result) const;
    /**
      @brief runs the transition rules over a segment std::list.
      @param scans the std::lists of segments - smoothed or unsmoothed.
      @param result std::vectors of transition rule matches and the field object ids they map to.
      */
    void filter(const SegmentedRegion& scans, map<COLOUR_CLASS, std::vector<ColourSegment> >& result) const;
    
    /**
      @brief Applies a single rule to a segmented region.
      @param scans the std::lists of segments - smoothed or unsmoothed.
      @param rule The transition rule to apply.
      @param matches the resulting std::list of transitions.
      */
    void checkRuleAgainstRegion(const SegmentedRegion& scans, const ColourTransitionRule& rule, std::vector<ColourSegment>& matches) const;
    /**
      @brief Applies a replacement rule to a triplet of segments.
      @param before the first segment.
      @param middle the second segment.
      @param after the last segment.
      @param replacement a reference to a std::vector of segments that should replace the middle segment.
      @param dir the scan direction (vertical or horizontal).
      */
    void applyReplacements(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after, std::vector<ColourSegment>& replacement, ScanDirection dir) const;
        
    /**
      @brief Joins any adjacent segments that are the same colour.
      @param line the std::list of segments.
      */
    void joinMatchingSegments(std::vector<ColourSegment>& line) const;

    /**
      @brief Loads the transition rules from a pair of files.
      @param filename the filename to load from, note that "_h.txt" and "_v.txt" will be appended to
             this to find the actual files.
      */
    void loadTransitionRules(std::string filename);
    /**
      @brief Loads the replacement rules from a pair of files.
      @param filename the filename to load from, note that "_h.txt" and "_v.txt" will be appended to
             this to find the actual files.
      */
    void loadReplacementRules(std::string filename);
    
public:
    static const bool PREFILTER_ON = true;
    
private:
    std::vector<ColourReplacementRule> replacement_rules_h;  //! @variable The std::list of horizontal replacement rules
    std::vector<ColourReplacementRule> replacement_rules_v;  //! @variable The std::list of vertical replacement rules
    std::vector<ColourTransitionRule> rules_h;               //! @variable The std::list of horizontal transition rules
    std::vector<ColourTransitionRule> rules_v;               //! @variable The std::list of vertical transition rules
    
};

#endif // SEGMENTFILTER_H
