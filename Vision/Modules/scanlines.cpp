/**
*   @name   ScanLines
*   @file   scanlines.cpp
*   @brief  generate horizontal and vertical scanlines.
*   @author David Budden
*   @date   23/03/2012
*/
#include "scanlines.h"
#include "debug.h"

void ScanLines::generateScanLines()
{
    #if VISION_SCAN_VERBOSITY > 1
        debug << "ScanLines::generateScanLines() - Begin" << endl;
    #endif
    VisionBlackboard *vbb = VisionBlackboard::getInstance();
    const vector<PointType>& horizon_points = vbb->getHorizonPoints();
    vector<unsigned int> horizontal_scan_lines;

    PointType left = horizon_points.front();
    PointType right = horizon_points.back();

    if(left.y >= vbb->getImageHeight())
        errorlog << "left: " << left.y << endl;
    
    if(right.y >= vbb->getImageHeight())
        errorlog << "right: " << right.y << endl;
    
    unsigned int bottom_horizontal_scan = (left.y + right.y) / 2;

    if(static_cast<int>(bottom_horizontal_scan) >= vbb->getImageHeight())
        errorlog << "cast avg: " << static_cast<int>(bottom_horizontal_scan) << " avg: " << bottom_horizontal_scan << endl;

    for (int y = static_cast<int>(bottom_horizontal_scan); y >= 0; y -= VERTICAL_SCANLINE_SKIP) {
        if(static_cast<unsigned int>(y) >= vbb->getImageHeight())
            errorlog << "sy: " << static_cast<unsigned int>(y) << " y: " << y << endl;
        horizontal_scan_lines.push_back(static_cast<unsigned int>(y));
    }
    
    vbb->setHorizontalScanlines(horizontal_scan_lines);
}

void ScanLines::classifyHorizontalScanLines()
{
    #if VISION_SCAN_VERBOSITY > 1
        debug << "ScanLines::classifyHorizontalScanLines() - Begin" << endl;
    #endif
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    const vector<unsigned int>& horizontal_scan_lines = vbb->getHorizontalScanlines();
    vector< vector<ColourSegment> > classifications;

    for(unsigned int i=0; i<horizontal_scan_lines.size(); i++) {
        classifications.push_back(classifyHorizontalScan(*vbb, img, horizontal_scan_lines.at(i)));
    }
    
    vbb->setHorizontalSegments(classifications);
}

void ScanLines::classifyVerticalScanLines()
{
    cout << "omg" << endl;
    #if VISION_SCAN_VERBOSITY > 1
        debug << "ScanLines::classifyVerticalScanLines() - Begin" << endl;
    #endif
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    const vector<PointType>& vertical_start_points = vbb->getHorizonPoints();
    vector< vector<ColourSegment> > classifications;

    for(unsigned int i=0; i<vertical_start_points.size(); i++) {
        classifications.push_back(classifyVerticalScan(*vbb, img, vertical_start_points.at(i)));
    }
    
    vbb->setVerticalSegments(classifications);
}

vector<ColourSegment> ScanLines::classifyHorizontalScan(const VisionBlackboard& vbb, const NUImage& img, unsigned int y)
{
    //simple and nasty first
    //ClassIndex::Colour previous, current, next
    int     start_pos = 0,
            x;
    const LookUpTable& lut = vbb.getLUT();
    ClassIndex::Colour start_colour = ClassIndex::getColourFromIndex(lut.classifyPixel(img(0,y))),
                        current_colour;
    ColourSegment segment;
    vector<ColourSegment> result;

    for(x = 0; x < img.getWidth(); x += HORIZONTAL_SKIP) {
        current_colour = ClassIndex::getColourFromIndex(lut.classifyPixel(img(x,y)));
        if(current_colour != start_colour) {
            //start of new segment
            //make new segment and push onto vector
            segment.set(PointType(start_pos, y), PointType(x, y), start_colour);
            result.push_back(segment);
            //start new segment
            start_colour = current_colour;
            start_pos = x;
        }
    }
    segment.set(PointType(start_pos, y), PointType(x, y), start_colour);
    result.push_back(segment);
    
#if VISION_SCAN_VERBOSITY > 1
    PointType end;
    for(int i=0; i<result.size(); i++) {
        cout << result.at(i).getStart() << " " << result.at(i).getEnd() << " " << (end==result.at(i).getStart()) << endl;
        end = result.at(i).getEnd();
    }
#endif
    return result;
}

vector<ColourSegment> ScanLines::classifyVerticalScan(const VisionBlackboard& vbb, const NUImage& img, const PointType &start)
{
    if(start.y >= img.getHeight() || start.x > img.getWidth())
        errorlog << start << endl;
    //simple and nasty first
    //ClassIndex::Colour previous, current, next
    const LookUpTable& lut = vbb.getLUT();
    ClassIndex::Colour start_colour = ClassIndex::getColourFromIndex(lut.classifyPixel(img(start.x,start.y))),
                        current_colour;
    ColourSegment segment;
    vector<ColourSegment> result;
    int     start_pos = start.y,
            x = start.x,
            y;
            
    for(y = start.y; y < img.getHeight(); y += VERTICAL_SKIP) {
        current_colour = ClassIndex::getColourFromIndex(lut.classifyPixel(img(x,y)));
        if(current_colour != start_colour) {
            //start of new segment
            //make new segment and push onto vector
            segment.set(PointType(x, start_pos), PointType(x, y), start_colour);
            result.push_back(segment);
            //start new segment
            start_colour = current_colour;
            start_pos = y;
        }
    }
    segment.set(PointType(x, start_pos), PointType(x, y), start_colour);
    result.push_back(segment);
    
    return result;
}
