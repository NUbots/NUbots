/**
*   @name   ScanLines
*   @file   scanlines.cpp
*   @brief  generate horizontal and vertical scanlines.
*   @author David Budden
*   @date   23/03/2012
*/
#include "scanlines.h"
#include "debug.h"
#include "Vision/visionconstants.h"

void ScanLines::generateScanLines()
{
    #if VISION_SCAN_VERBOSITY > 1
        debug << "ScanLines::generateScanLines() - Begin" << endl;
    #endif
    VisionBlackboard *vbb = VisionBlackboard::getInstance();
    vector<unsigned int> horizontal_scan_lines;
    const vector<Vector2<double> >& horizon_points = vbb->getGreenHorizon().getInterpolatedPoints();   //need this to get the left and right

    Vector2<double> left = horizon_points.front();
    Vector2<double> right = horizon_points.back();

    if(left.y >= vbb->getImageHeight())
        errorlog << "left: " << left.y << endl;
    
    if(right.y >= vbb->getImageHeight())
        errorlog << "right: " << right.y << endl;
    
    //unsigned int bottom_horizontal_scan = (left.y + right.y) / 2;
    unsigned int bottom_horizontal_scan = vbb->getImageHeight() - 1;    //we need h-scans under the GH for field lines

    if(static_cast<int>(bottom_horizontal_scan) >= vbb->getImageHeight())
        errorlog << "cast avg: " << static_cast<int>(bottom_horizontal_scan) << " avg: " << bottom_horizontal_scan << endl;

    for (int y = static_cast<int>(bottom_horizontal_scan); y >= 0; y -= VisionConstants::HORIZONTAL_SCANLINE_SPACING) {
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
    #if VISION_SCAN_VERBOSITY > 1
        debug << "ScanLines::classifyVerticalScanLines() - Begin" << endl;
    #endif
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    const vector<Vector2<double> >& vertical_start_points = vbb->getGreenHorizon().getInterpolatedSubset(VisionConstants::VERTICAL_SCANLINE_SPACING);
    vector< vector<ColourSegment> > classifications;

    for(unsigned int i=0; i<vertical_start_points.size(); i++) {
        classifications.push_back(classifyVerticalScan(*vbb, img, vertical_start_points.at(i)));
    }
    
    vbb->setVerticalSegments(classifications);
}

vector<ColourSegment> ScanLines::classifyHorizontalScan(const VisionBlackboard& vbb, const NUImage& img, unsigned int y)
{
    //simple and nasty first
    //Colour previous, current, next
    int     start_pos = 0,
            x;
    const LookUpTable& lut = vbb.getLUT();
    Colour start_colour = getColourFromIndex(lut.classifyPixel(img(0,y))),
                        current_colour;
    ColourSegment segment;
    vector<ColourSegment> result;

    for(x = 0; x < img.getWidth(); x++) {
        current_colour = getColourFromIndex(lut.classifyPixel(img(x,y)));
        if(current_colour != start_colour) {
            //start of new segment
            //make new segment and push onto vector
            segment.set(Vector2<double>(start_pos, y), Vector2<double>(x, y), start_colour);
            result.push_back(segment);
            //start new segment
            start_colour = current_colour;
            start_pos = x;
        }
    }
    segment.set(Vector2<double>(start_pos, y), Vector2<double>(x-1, y), start_colour);
    result.push_back(segment);
    
    #if VISION_SCAN_VERBOSITY > 1
        Vector2<double> end;
        for(int i=0; i<result.size(); i++) {
            debug << result.at(i).getStart() << " " << result.at(i).getEnd() << " " << (end==result.at(i).getStart()) << endl;
            end = result.at(i).getEnd();
        }
    #endif
    return result;
}

vector<ColourSegment> ScanLines::classifyVerticalScan(const VisionBlackboard& vbb, const NUImage& img, const Vector2<double> &start)
{
    if(start.y >= img.getHeight() || start.x > img.getWidth())
        errorlog << start << endl;
    //simple and nasty first
    //Colour previous, current, next
    const LookUpTable& lut = vbb.getLUT();
    Colour start_colour = getColourFromIndex(lut.classifyPixel(img(start.x,start.y))),
                        current_colour;
    ColourSegment segment;
    vector<ColourSegment> result;
    int     start_pos = start.y,
            x = start.x,
            y;
            
    for(y = start.y; y < img.getHeight(); y++) {
        current_colour = getColourFromIndex(lut.classifyPixel(img(x,y)));
        if(current_colour != start_colour) {
            //start of new segment
            //make new segment and push onto vector
            segment.set(Vector2<double>(x, start_pos), Vector2<double>(x, y), start_colour);
            result.push_back(segment);
            //start new segment
            start_colour = current_colour;
            start_pos = y;
        }
    }
    segment.set(Vector2<double>(x, start_pos), Vector2<double>(x, y), start_colour);
    result.push_back(segment);
    
    return result;
}
