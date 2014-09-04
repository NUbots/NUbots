#ifndef SEGMENTEDREGION_H
#define SEGMENTEDREGION_H

#include <vector>

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/basicvisiontypes.h"

using std::vector;
using Vision::ScanDirection;

class SegmentedRegion
{
friend class SegmentFilter;
    
public:
    SegmentedRegion();
    SegmentedRegion(const SegmentedRegion& other);
    SegmentedRegion(const std::vector<std::vector<ColourSegment> >& segmented_scans, ScanDirection direction);
    
    /**
      * Sets the segments and direction of this region.
      * @param segmented_scans A 2D vector of segments.
      * @param direction The alignment of the segments in this region (vertical or horizontal).
      */
    void set(const std::vector<std::vector<ColourSegment> >& segmented_scans, ScanDirection direction);

    bool empty() const {return m_segmented_scans.empty();}
    
    //consider removing later and replacing with iterator
    //! Returns a const reference to the segments.
    const std::vector<std::vector<ColourSegment> >& getSegments() const;

    //! Returns the number of segments in the region.
    size_t getNumberOfScans() const;
    //! Returns the alignment of the scans.
    ScanDirection getDirection() const;

private:
    std::vector< std::vector<ColourSegment> > m_segmented_scans;  //! @variable The segments in this region.
    ScanDirection m_direction;  //! The alignment of the scans in this region.
};

#endif // SEGMENTEDREGION_H
