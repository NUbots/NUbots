#ifndef SEGMENTEDREGION_H
#define SEGMENTEDREGION_H

#include <vector>
#include "VisionTypes/coloursegment.h"
#include "basicvisiontypes.h"

using std::vector;
using VisionID::ScanDirection;

class SegmentedRegion
{
friend class SegmentFilter;
    
public:
    SegmentedRegion();
    SegmentedRegion(const SegmentedRegion& other);
    SegmentedRegion(const vector<vector<ColourSegment> >& segmented_scans, ScanDirection direction);
    
    void set(const vector<vector<ColourSegment> >& segmented_scans, ScanDirection direction);
    
    //consider removing later and replacing with iterator
    const vector<vector<ColourSegment> >& getSegments() const;
    size_t getNumberOfScans() const;
    ScanDirection getDirection() const;
    
    vector<ColourSegment>::const_iterator iteratorAt(size_t i) const;
    
private:
    vector< vector<ColourSegment> > m_segmented_scans;
    ScanDirection m_direction;
};

#endif // SEGMENTEDREGION_H
