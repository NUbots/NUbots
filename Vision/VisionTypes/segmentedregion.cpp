#include "segmentedregion.h"

SegmentedRegion::SegmentedRegion()
{
}

SegmentedRegion::SegmentedRegion(const SegmentedRegion& other)
{
    SegmentedRegion(other.m_segmented_scans, other.m_direction);
}

SegmentedRegion::SegmentedRegion(const vector<vector<ColourSegment> >& segmented_scans, ScanDirection direction)
{
    set(segmented_scans, direction);
}

void SegmentedRegion::set(const vector<vector<ColourSegment> >& segmented_scans, ScanDirection direction)
{
    m_segmented_scans = segmented_scans; //vector assignment operator copies elements
    m_direction = direction;
}

const vector<vector<ColourSegment> >& SegmentedRegion::getSegments() const 
{
    return m_segmented_scans;
} 

size_t SegmentedRegion::getNumberOfScans() const
{
    return m_segmented_scans.size();
}

ScanDirection SegmentedRegion::getDirection() const
{
    return m_direction;
}
