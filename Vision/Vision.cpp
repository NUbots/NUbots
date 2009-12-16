/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
  @author Steven Nicklin
*/

#include "Vision.h"
#include "Tools/Image/NUImage.h"
#include "ClassificationColours.h"
#include <QDebug>
#include <boost/circular_buffer.hpp>

Vision::Vision()
{
    return;
}

Vision::~Vision()
{
    return;
}

unsigned char Vision::classifyPixel(int x, int y)
{
    pixels::Pixel* temp = &currentImage->image[y][x];
    return currentLookupTable[(temp->y<<16) + (temp->cb<<8) + temp->cr];
}

void Vision::classifyImage(ClassifiedImage &target, const NUimage* sourceImage, const unsigned char *lookUpTable)
{   
    target.setImageDimensions(sourceImage->width(),sourceImage->height());
    currentImage = sourceImage;
    currentLookupTable = lookUpTable;
    for (int y = 0; y < sourceImage->height(); y++)
    {
        for (int x = 0; x < sourceImage->width(); x++)
        {
            target.image[y][x] = classifyPixel(x,y);
        }
    }
    return;
}

std::vector< Vector2<int> > Vision::findGreenBorderPoints(const NUimage* sourceImage, const unsigned char *lookUpTable, int scanSpacing, Horizon* horizonLine)
{
    std::vector< Vector2<int> > results;
    currentImage = sourceImage;
    currentLookupTable = lookUpTable;
    int yStart;
    int consecutiveGreenPixels = 0;
    for (int x = 0; x < sourceImage->width(); x+=scanSpacing)
    {
        yStart = (int)horizonLine->findYFromX(x);
        if(yStart > sourceImage->height()) continue;
        if(yStart < 0) yStart = 0;
        consecutiveGreenPixels = 0;
        for (int y = yStart; y < sourceImage->height(); y++)
        {
            if(classifyPixel(x,y) == ClassIndex::green)
            {
                consecutiveGreenPixels++;
            }
            else
            {
                consecutiveGreenPixels = 0;
            }
            if(consecutiveGreenPixels >= 3)
            {
                results.push_back(Vector2<int>(x,y-consecutiveGreenPixels+1));
                break;
            }
        }
    }
    return results;
}

#define LEFT_OF(x0, x1, x2) ((x1.x-x0.x)*(-x2.y+x0.y)-(x2.x-x0.x)*(-x1.y+x0.y) > 0)

std::vector<Vector2<int> > Vision::getConvexFieldBorders(std::vector<Vector2<int> >& fieldBorders)
{
  //Andrew's Monotone Chain Algorithm to compute the upper hull
  std::vector<Vector2<int> > hull;
  if(!fieldBorders.size()) return hull;
  const std::vector<Vector2<int> >::const_iterator pmin = fieldBorders.begin(),
                                                   pmax = fieldBorders.end()-1;
  hull.push_back(*pmin);
  for(std::vector<Vector2<int> >::const_iterator pi = pmin + 1; pi != pmax+1; pi++)
  {
    if(!LEFT_OF((*pmin), (*pmax), (*pi)) && pi != pmax)
      continue;

    while((int)hull.size() > 1)
    {
      const std::vector<Vector2<int> >::const_iterator p1 = hull.end() - 1,
                                                       p2 = hull.end() - 2;
      if(LEFT_OF((*p1), (*p2), (*pi)))
        break;
      hull.pop_back();
    }
    hull.push_back(*pi);
  }
  return hull;
}

std::vector<Vector2<int> > Vision::interpolateBorders(std::vector<Vector2<int> >& fieldBorders, int scanSpacing)
{
    std::vector<Vector2<int> > interpolatedBorders;
    if(!fieldBorders.size()) return interpolatedBorders;
    std::vector<Vector2<int> >::const_iterator nextPoint = fieldBorders.begin();
    std::vector<Vector2<int> >::const_iterator prevPoint = nextPoint++;

    int x = prevPoint->x;
    Vector2<int> deltaPoint, temp;
    for (; nextPoint != fieldBorders.end(); nextPoint++)
    {
        deltaPoint = (*nextPoint) - (*prevPoint);
        for (; x <= nextPoint->x; x+=scanSpacing)
        {
            temp.x = x;
            temp.y = (x - prevPoint->x) * deltaPoint.y / deltaPoint.x + prevPoint->y;
            if (temp.y < 0) temp.y = 0;
            if (temp.y >= currentImage->height()) temp.y = currentImage->height() - 1;
            interpolatedBorders.push_back(temp);
        }
        prevPoint = nextPoint;
    }
    return interpolatedBorders;
}



ClassifiedSection* Vision::verticalScan(std::vector<Vector2<int> >&fieldBorders,int scanSpacing)
{
    //std::vector<Vector2<int> > scanPoints;
    ClassifiedSection* scanArea = new ClassifiedSection(ClassifiedSection::DOWN);
    if(!fieldBorders.size()) return scanArea;
    std::vector<Vector2<int> >::const_iterator nextPoint = fieldBorders.begin();
    //std::vector<Vector2<int> >::const_iterator prevPoint = nextPoint++; //This iterator is unused
    int x = 0;
    int y = 0;
    int fullLineLength = 0;
    int halfLineLength = 0;
    int quarterLineLength = 0;
    int midX = 0;
    int skip = int(scanSpacing/2);

    Vector2<int> temp;
    for (; nextPoint != fieldBorders.end(); nextPoint++)
    {
        x = nextPoint->x;
        y = nextPoint->y;

        //!Create Full ScanLine
        temp.x = x;
        temp.y = y;

        fullLineLength = int(currentImage->height() - y);
        ScanLine* tempScanLine = new ScanLine(temp, fullLineLength);
        scanArea->addScanLine(tempScanLine);

        //!Create half ScanLine
        midX = x-skip;
        temp.x = midX;
        halfLineLength = int((currentImage->height() - y)/2);
        ScanLine* tempMidScanLine = new ScanLine(temp,halfLineLength);
        scanArea->addScanLine(tempMidScanLine);

        //!Create Quarter ScanLines
        temp.x = int(midX - skip/2);
        quarterLineLength = int((currentImage->height() - y)/4);
        ScanLine* tempLeftQuarterLine = new ScanLine(temp,quarterLineLength);
        scanArea->addScanLine(tempLeftQuarterLine);
        temp.x = int(midX + skip/2);
        ScanLine* tempRightQuarterLine = new ScanLine(temp,quarterLineLength);
        scanArea->addScanLine(tempRightQuarterLine);
    }

    //!Generate the last Lines:
    midX = fieldBorders.back().x+skip;
    y = fieldBorders.back().y;
    temp.x = midX;
    temp.y = y;
    ScanLine* tempMidScanLine = new ScanLine(temp,halfLineLength);
    scanArea->addScanLine(tempMidScanLine);
    temp.x = midX-skip/2;
    temp.y = y;
    ScanLine* tempLeftQuarterLine = new ScanLine(temp,quarterLineLength);
    scanArea->addScanLine(tempLeftQuarterLine);
    temp.x = midX+skip/2;
    temp.y = y;
    ScanLine* tempRightQuarterLine = new ScanLine(temp,quarterLineLength);
    scanArea->addScanLine(tempRightQuarterLine);

    return scanArea;
}

ClassifiedSection* Vision::horizontalScan(std::vector<Vector2<int> >&fieldBorders,int scanSpacing)
{
    ClassifiedSection* scanArea = new ClassifiedSection(ClassifiedSection::RIGHT);
    if(!currentImage) return scanArea;
    Vector2<int> temp;
    //! Case for No FieldBoarders
    if(!fieldBorders.size())
    {

        for(int y = 0; y < currentImage->height(); y = y + scanSpacing*2)
        {
            temp.x = 0;
            temp.y = y;
            ScanLine* tempScanLine = new ScanLine(temp,currentImage->width());
            scanArea->addScanLine(tempScanLine);
        }
        return scanArea;
    }

    //! Find the minimum Y, and scan above the field boarders

    std::vector<Vector2<int> >::const_iterator nextPoint = fieldBorders.begin();
    std::vector<Vector2<int> >::const_iterator prevPoint = nextPoint++;
    int minY = currentImage->height();
    int maxY = 0;
    for (; nextPoint != fieldBorders.end(); nextPoint++)
    {
        if(nextPoint->y < minY)
        {
            minY = nextPoint->y;
        }
        if(nextPoint->y >maxY)
        {
            maxY = nextPoint->y;
        }
    }

    //! Then calculate horizontal scanlines above the field boarder
    //! Generate Scan pattern for above the max of green boarder.
    for(int y = minY; y > 0; y = y - scanSpacing)
    {
        temp.x =0;
        temp.y = y;
        ScanLine* tempScanLine = new ScanLine(temp,currentImage->width());
        scanArea->addScanLine(tempScanLine);
    }
    //! Generate Scan Pattern for in between the max and min of green horizon.
    for(int y = minY; y < maxY; y = y + scanSpacing*2)
    {
        temp.x =0;
        temp.y = y;
        ScanLine* tempScanLine = new ScanLine(temp,currentImage->width());
        scanArea->addScanLine(tempScanLine);
    }
    return scanArea;
}


void Vision::ClassifyScanArea(ClassifiedSection* scanArea)
{
    int direction = scanArea->getDirection();
    int numOfLines = scanArea->getNumberOfScanLines();
    int lineLength = 0;
    ScanLine* tempLine;
    TransitionSegment* tempTransition ;
    Vector2<int> currentPoint;
    Vector2<int> tempStartPoint;
    Vector2<int> tempEndPoint;
    unsigned char beforeColour = 0; //!< Colour Before the segment
    unsigned char afterColour = 0;  //!< Colour in the next Segment
    unsigned char currentColour = 0; //!< colour in the current segment
    //! initialising circular buffer
    int bufferSize = 3;
    boost::circular_buffer<unsigned char> colourBuff(bufferSize);

    for (int i = 0; i < bufferSize; i++)
    {
        colourBuff.push_back(0);
    }

    for (int i = 0; i < numOfLines; i++)
    {
        tempLine = scanArea->getScanLine(i);
        Vector2<int> startPoint = tempLine->getStart();
        lineLength = tempLine->getLength();
        tempStartPoint = startPoint;
        //! No point in scanning lines less then the buffer size
        if(lineLength < bufferSize) continue;

        for(int j = 0; j < lineLength; j++)
        {
            if(direction == ClassifiedSection::DOWN)
            {
                currentPoint.x = startPoint.x;
                currentPoint.y = startPoint.y + j;
            }
            else if (direction == ClassifiedSection::RIGHT)
            {
                currentPoint.x = startPoint.x + j;
                currentPoint.y = startPoint.y;
            }
            else if(direction == ClassifiedSection::UP)
            {
                currentPoint.x = startPoint.x;
                currentPoint.y = startPoint.y - j;
            }
            else if(direction == ClassifiedSection::LEFT)
            {
                currentPoint.x = startPoint.x - j;
                currentPoint.y = startPoint.y;
            }

            afterColour = classifyPixel(currentPoint.x,currentPoint.y);
            colourBuff.push_back(currentColour);
            if(checkIfBufferSame(colourBuff))
            {
                if(currentColour != afterColour)
                {
                    //! Transition detected: Generate new segment and add to the line
                    tempTransition = new TransitionSegment(tempStartPoint, currentPoint, beforeColour, currentColour, afterColour);
                    tempLine->addSegement(tempTransition);
                    tempStartPoint = currentPoint;
                    beforeColour = currentColour;
                    currentColour = afterColour;
                    qDebug() << "Found "<<ClassIndex::getColourNameFromIndex(currentColour) << " segment.";
                }
            }


        }
        qDebug() << tempLine->getNumberOfSegments() <<" number of segments on line "<< i;
    }
    return;
}

int Vision::countRobots(std::vector<Vector2<int> > &fieldBorders)
{
    int robotCount = 0;
    boost::circular_buffer<int> cb(3);
    if(!fieldBorders.size()) return robotCount;
    if (!currentImage || !currentLookupTable)
    {
        qDebug() << "currentImage not set or currentLookUpTable not set";
        return -2;
    }
    //   list of pairs of coordinates to denote the end points for a candidate region
    std::vector<Vector2<int> > candidateRegions;
    std::vector<Vector2<int> >::const_iterator currentPoint = fieldBorders.begin();

    std::vector<Vector2<int> > tempRegion;
    Vector2<int> regionStart, regionStop;

    //int x = 0;
    //int y = 0;
    int p_x = currentPoint->y; //previous x
    int p_y = currentPoint->y; //previous y
    int c_x = 0; //current x
    int c_y = 0; //current y





    //
    //Trace along the field border looking for Green-White-Green transitions
    //and marking these regions as candidate robots
    //
    for(;currentPoint != fieldBorders.end(); currentPoint++)
    {
        c_x = currentPoint->x;
        c_y = currentPoint->y;


        if (p_x != c_x && p_y != c_y)
        {
            //
            //Scan the points in between previous point
            //and current point using scanSpacing
            //
            //int m = (c_y-p_y)/(c_x-p_x);
            //int b = p_y - m*p_x;
            //for (x = p_x, y = p_y; x < c_x; x += scanSpacing)
            //{
               //y = m*x + b;
               qDebug() << "(" << c_x << "," << c_y << ")";

               //
               //A circular buffer must initially fill up to a starting colour.
               //When the buffer fully becomes the new colour a new region has started
               //If this new region is white store the start point and continue
               //until it triggers a different colour. Mark this as the end and store
               //both points as endpoints for a candidate region.
               //

               //if ( candidate region found )
               //{
               //   robotCount++;
               //}
            //}
        }



        p_x = c_x;
        p_y = c_y;
    }



    //
    //Candidate White regions have to be above a threshold width
    //which will be based upon distance/orientation
    //

    //
    //Candidate white region also has to be less than a threshold
    //width which will be based upondistance/orientation
    //

    //
    //Determine if candidate region is a line marking and reject region
    //


    return robotCount;
}

bool Vision::checkIfBufferSame(boost::circular_buffer<unsigned char> cb)
{

    unsigned char currentClass = cb[0];
    for (unsigned int i = 1; i < cb.size(); i++)
    {
        if(cb[i] != currentClass)
        {
            return false;
        }
    }

    return true;

}
