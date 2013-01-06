#ifndef SEGMENTEDREGION_H
#define SEGMENTEDREGION_H

#include <vector>

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/basicvisiontypes.h"

using std::vector;
using VisionID::ScanDirection;

class SegmentedRegion
{
friend class SegmentFilter;
    
public:
    SegmentedRegion();
    SegmentedRegion(const SegmentedRegion& other);
    SegmentedRegion(const vector<vector<ColourSegment> >& segmented_scans, ScanDirection direction);
    
    /**
      * Sets the segments and direction of this region.
      * @param segmented_scans A 2D vector of segments.
      * @param direction The alignment of the segments in this region (vertical or horizontal).
      */
    void set(const vector<vector<ColourSegment> >& segmented_scans, ScanDirection direction);

    bool empty() const {return m_segmented_scans.empty();}
    
    //consider removing later and replacing with iterator
    //! Returns a const reference to the segments.
    const vector<vector<ColourSegment> >& getSegments() const;

    //! Returns the number of segments in the region.
    size_t getNumberOfScans() const;
    //! Returns the alignment of the scans.
    ScanDirection getDirection() const;
    
    //! Returns an iterator over the region beginning at the given scan.
    vector<ColourSegment>::const_iterator iteratorAt(size_t i) const;
    
private:
    vector< vector<ColourSegment> > m_segmented_scans;  //! @variable The segments in this region.
    ScanDirection m_direction;  //! The alignment of the scans in this region.
};

#endif // SEGMENTEDREGION_H
