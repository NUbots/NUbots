/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
  @author Steven Nicklin
*/

#include "Vision.h"
#include "Tools/Image/NUImage.h"
#include "Tools/Math/Line.h"
#include "ClassificationColours.h"

#include <QDebug>
#include <boost/circular_buffer.hpp>
#include <queue>
#include <algorithm>

Vision::Vision()
{
    AllFieldObjects = new FieldObjects();
    classifiedCounter = 0;
    return;
}

Vision::~Vision()
{

    return;
}

unsigned char Vision::classifyPixel(int x, int y)
{
    classifiedCounter++;
    pixels::Pixel* temp = &currentImage->image[y][x];
    return currentLookupTable[(temp->y<<16) + (temp->cb<<8) + temp->cr];
}

void Vision::classifyImage(ClassifiedImage &target, const NUimage* sourceImage, const unsigned char *lookUpTable)
{   
    int tempClassCounter = classifiedCounter;
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
    classifiedCounter = tempClassCounter;
    return;
}

std::vector< Vector2<int> > Vision::findGreenBorderPoints(const NUimage* sourceImage, const unsigned char *lookUpTable, int scanSpacing, Horizon* horizonLine)
{
    classifiedCounter = 0;
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
    //! Case for No FieldBorders
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
    /*//! Generate Scan Pattern under green horizon
    for(int y = minY; y < currentImage->height(); y = y + scanSpacing/2)
    {
        temp.x =0;
        temp.y = y;
        ScanLine* tempScanLine = new ScanLine(temp,currentImage->width());
        scanArea->addScanLine(tempScanLine);
    }*/
    return scanArea;
}
/* removed temporarliy
std::vector< ClassifiedSection > Vision::robotScanAreas(std::vector<RobotCandidate> robotCandidates, std::vector<Vector2<int> >&fieldBorders, Horizon horizonLine)
{
    int scanSpacing = 4;
    std::vector< ClassifiedSection > robotScanAreas;
    if(!robotCandidates.size()) return robotScanAreas;
    std::vector< RobotCandidate >::const_iterator nextRobot = robotCandidates.begin();
    int min_x = 0;
    int max_x = 0;
    int max_y = 0;
    for (;nextRobot != robotCandidates.end(); nextRobot++)
    {
        ClassifiedSection* scanArea = new ClassifiedSection(ClassifiedSection::DOWN);

        min_x = nextRobot->getTopLeft().x;
        max_x = nextRobot->getBottomRight().x;
        max_y = nextRobot->getBottomRight().y;
        Vector2<int> temp;

        if ( (max_x - min_x)*(max_y - (int)horizonLine.findYFromX((double)min_x)) > currentImage->width()*currentImage->height()/4 )
        {
            scanSpacing = 4;
        }
        else if ( (max_x - min_x)*(max_y - (int)horizonLine.findYFromX((double)min_x)) > currentImage->width()*currentImage->height()/8 )
        {
            scanSpacing = 3;
        }
        else if ( (max_x - min_x)*(max_y - (int)horizonLine.findYFromX((double)min_x)) > currentImage->width()*currentImage->height()/16 )
        {
            scanSpacing = 2;
        }
        else
        {
            scanSpacing = 1;
        }
        //scanSpacing = 2;
        for ( int x = min_x; x <= max_x; x += scanSpacing)
        {
            temp.x = x;
            temp.y = (int)horizonLine.findYFromX((double)x);
            if (temp.y < 0) temp.y = 0;
            //TODO
            //Have option of only scanning from green border to horizon
            //also compare the horizontal scan from feet to horizon
            //qDebug() << findYFromX(fieldBorders,x) << " - " << temp.y << " = " << (findYFromX(fieldBorders,x)- temp.y);
            //ScanLine* tempScanLine = new ScanLine(temp, (findYFromX(fieldBorders,x) - temp.y));
            //qDebug() << "ScanLine Created...";
            ScanLine* tempScanLine = new ScanLine(temp, max_y - temp.y);
            scanArea->addScanLine(tempScanLine);
            //qDebug() << "ScanLine added...";
        }
        ClassifyScanArea(scanArea);
        //qDebug() << "ScanArea Classified...";
        robotScanAreas.push_back(*scanArea);
    }
    return robotScanAreas;
}
*/
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
    unsigned char currentColour = 0; //!< Colour in the current segment
    //! initialising circular buffer
    int bufferSize = 1;
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
            colourBuff.push_back(afterColour);

            if(j == lineLength-1)
            {
                //! End Of Screen detected: Generate new segment and add to the line
                if(!(currentColour == ClassIndex::green || currentColour == ClassIndex::unclassified))
                {
                    tempTransition = new TransitionSegment(tempStartPoint, currentPoint, beforeColour, currentColour, afterColour);
                    tempLine->addSegement(tempTransition);
                    int spacing = 16;
                    int segmentlength = currentPoint.y-tempStartPoint.y;
                    if(abs(segmentlength)>spacing)
                    {
                        CloselyClassifyScanline(tempLine, tempStartPoint,currentColour,segmentlength, spacing, direction);
                    }
                }
                tempStartPoint = currentPoint;
                beforeColour = ClassIndex::unclassified;
                currentColour = afterColour;
                continue;
            }

            if(checkIfBufferSame(colourBuff))
            {
                if(currentColour != afterColour)
                {
                    //! Transition detected: Generate new segment and add to the line
                    //Adjust the position:
                    if(!(currentColour == ClassIndex::green || currentColour == ClassIndex::unclassified ))
                    {
                        //SHIFTING THE POINTS TO THE START OF BUFFER:
                        if(direction == ClassifiedSection::DOWN)
                        {
                            currentPoint.x = startPoint.x;
                            currentPoint.y = startPoint.y + j;// - bufferSize;
                        }
                        else if (direction == ClassifiedSection::RIGHT)
                        {
                            currentPoint.x = startPoint.x + j;// - bufferSize;
                            currentPoint.y = startPoint.y;
                        }
                        else if(direction == ClassifiedSection::UP)
                        {
                            currentPoint.x = startPoint.x;
                            currentPoint.y = startPoint.y - j;// + bufferSize;
                        }
                        else if(direction == ClassifiedSection::LEFT)
                        {
                            currentPoint.x = startPoint.x - j;// + bufferSize;
                            currentPoint.y = startPoint.y;
                        }
                        tempTransition = new TransitionSegment(tempStartPoint, currentPoint, beforeColour, currentColour, afterColour);
                        tempLine->addSegement(tempTransition);
                        //SCAN FOR OTHER SEGMENTS:
                        int spacing = 16;
                        if(abs(currentPoint.y-tempStartPoint.y)>spacing)
                        {
                            int length = currentPoint.y-tempStartPoint.y;
                            CloselyClassifyScanline(tempLine, tempStartPoint,currentColour,length, spacing, direction);
                        }
                    }
                    tempStartPoint = currentPoint;
                    beforeColour = currentColour;
                    currentColour = afterColour;

                }
            }
        }

    }
    return;
}

void Vision::CloselyClassifyScanline(ScanLine* tempLine, Vector2<int> tempStartPoint, unsigned char currentColour, int length, int spacings, int direction)
{
    if((direction == ClassifiedSection::DOWN || direction == ClassifiedSection::UP))
    {
        Vector2<int> tempSubEndPoint;
        Vector2<int> tempSubStartPoint;
        unsigned char subAfterColour;
        unsigned char subBeforeColour;
        unsigned char tempColour = currentColour;
        for(int k =0; k < abs(length); k = k+spacings)
        {
            tempSubEndPoint.y = tempStartPoint.y+k;
            tempSubStartPoint.y = tempStartPoint.y+k;
            int tempsubPoint = tempStartPoint.x;
            unsigned char  tempColour = currentColour;;

            while(tempColour == currentColour)
            {
                if(tempsubPoint+1 > currentImage->width()) break;

                tempsubPoint++;

                if(tempStartPoint.y+k < currentImage->height() && tempStartPoint.y+k > 0 &&
                   tempsubPoint < currentImage->width() && tempsubPoint > 0)
                {
                    tempColour= classifyPixel(tempsubPoint,tempStartPoint.y+k);
                }
                else
                {
                    break;
                }
            }
            tempSubEndPoint.x = tempsubPoint;
            subAfterColour = tempColour;
            tempsubPoint = tempStartPoint.x;
            tempColour = currentColour;

            while(tempColour == currentColour)
            {
                if(tempsubPoint-1 < 0) break;
                tempsubPoint--;
                if(tempStartPoint.y+k < currentImage->height() && tempStartPoint.y+k > 0
                   && tempsubPoint < currentImage->width() && tempsubPoint > 0)
                {
                    tempColour = classifyPixel(tempsubPoint,tempStartPoint.y+k);
                }
                else
                {
                    break;
                }
            }
            tempSubStartPoint.x = tempsubPoint;
            subBeforeColour = tempColour;
            //THEN ADD TO LINE

            TransitionSegment* tempTransition = new TransitionSegment(tempSubStartPoint, tempSubEndPoint, subBeforeColour , currentColour, subAfterColour);
            tempLine->addSegement(tempTransition);



    /*
    else if (direction == ClassifiedSection::RIGHT || direction == ClassifiedSection::LEFT)
    {
    //SCAN UNTIL TRANSITION IN BOTH UP AND DOWN
    //THEN ADD TO LINE
    int tempPoint = currentPoint.y;
    unsigned char nextsubColour;
    while(checkIfBufferSame(colourBuff2))
    {
        tempPoint++;
        nextsubColour = classifyPixel(currentPoint.x,tempPoint);
        colourBuff.push_back(nextsubColour);
    }
    tempSubEndPoint.x = currentPoint.x;
    tempSubEndPoint.y = tempPoint;
    subAfterColour = nextsubColour;
    tempPoint = currentPoint.y;
    colourBuff2.push_back(currentColour);
    while(checkIfBufferSame(colourBuff2))
    {
        tempPoint--;
        unsigned char nextColour = classifyPixel(currentPoint.x,tempPoint);
        colourBuff.push_back(nextColour);
    }
    tempSubStartPoint.x = currentPoint.x;
    tempSubStartPoint.y = tempPoint;
    subBeforeColour = nextsubColour;
    //THEN ADD TO LINE
    tempTransition = new TransitionSegment(tempSubStartPoint, tempSubEndPoint, subBeforeColour , currentColour, subAfterColour);
    tempLine->addSegement(tempTransition);
    }
    */
    }
    }
}

std::vector<ObjectCandidate> Vision::classifyCandidates(
                                        std::vector< TransitionSegment > segments,
                                        std::vector<Vector2<int> >&fieldBorders,
                                        std::vector<unsigned char> validColours,
                                        int spacing,
                                        float min_aspect, float max_aspect, int min_segments,
                                        tCLASSIFY_METHOD method)
{
    switch(method)
    {
        case PRIMS:
            return classifyCandidatesPrims(segments, fieldBorders, validColours, spacing, min_aspect, max_aspect, min_segments);
        break;
        case DBSCAN:
            return classifyCandidatesDBSCAN(segments, fieldBorders, validColours, spacing, min_aspect, max_aspect, min_segments);
        break;
        default:
            return classifyCandidatesPrims(segments, fieldBorders, validColours, spacing,  min_aspect, max_aspect, min_segments);
        break;
    }

}

std::vector<ObjectCandidate> Vision::classifyCandidatesPrims(std::vector< TransitionSegment > segments,
                                        std::vector<Vector2<int> >&fieldBorders,
                                        std::vector<unsigned char> validColours,
                                        int spacing,
                                        float min_aspect, float max_aspect, int min_segments)
{
    //! Overall runtime O( (K*(2*M^3 + M^2) + N*(LogN + 1) )
    std::vector<ObjectCandidate> candidateList;

    const int VERT_JOIN_LIMIT = 3;
    const int HORZ_JOIN_LIMIT = 1;


    if (!segments.empty())
    {
        //! Sorting O(N*logN)
        sort(segments.begin(), segments.end(), Vision::sortTransitionSegments);

        std::queue<int> qUnprocessed;
        unsigned int rawSegsLeft = segments.size();
        int nextRawSeg = 0;

        bool isSegUsed [segments.size()];

        //! Removing invalid colours O(N)
        for (unsigned int i = 0; i < segments.size(); i++)
        {
            //may have non-robot colour segments, so pre-mark them as used
            if (isValidColour(segments.at(i).getColour(), validColours))
            {
                //qDebug() << ClassIndex::getColourNameFromIndex(segments.at(i).getColour()) << isRobotColour(segments.at(i).getColour());
                isSegUsed[i] = false;
                //qDebug() <<  "(" << segments.at(i).getStartPoint().x << "," << segments.at(i).getStartPoint().y << ")-("<< segments.at(i).getEndPoint().x << "," << segments.at(i).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(i).getColour()) << "]";
            }
            else
            {
                rawSegsLeft--;
                isSegUsed[i] = true;
            }

        }

        //! For all valid segments O(M)
        while(rawSegsLeft)
        {

            //Roll through and find first unused segment
            nextRawSeg = 0;

            //! Find next unused segment O(M)
            while(isSegUsed[nextRawSeg] && nextRawSeg < segments.size()) nextRawSeg++;
            //Prime unprocessed segment queue to build next candidate
            qUnprocessed.push(nextRawSeg);
            //take away from pool of raw segs
            isSegUsed[nextRawSeg] = true;
            rawSegsLeft--;

            int min_x, max_x, min_y, max_y, segCount;
            int * colourHistogram = new int[validColours.size()];
            min_x = segments.at(nextRawSeg).getStartPoint().x;
            max_x = segments.at(nextRawSeg).getStartPoint().x;
            min_y = segments.at(nextRawSeg).getStartPoint().y;
            max_y = segments.at(nextRawSeg).getEndPoint().y;
            segCount = 0;
            for (int i = 0; i < validColours.size(); i++)  colourHistogram[i] = 0;

            //! For all unprocessed joined segment in a candidate O(M)
            //Build candidate
            while (!qUnprocessed.empty())
            {
                unsigned int thisSeg;
                thisSeg = qUnprocessed.front();
                qUnprocessed.pop();
                segCount++;
                for (int i = 0; i < validColours.size(); i++)
                {
                    if ( segments.at(thisSeg).getColour() == validColours.at(i) && validColours.at(i) != ClassIndex::white)
                    {
                        colourHistogram[i] += 1;
                        i = validColours.size();
                    }
                }

                if ( min_x > segments.at(thisSeg).getStartPoint().x)
                    min_x = segments.at(thisSeg).getStartPoint().x;
                if ( max_x < segments.at(thisSeg).getStartPoint().x)
                    max_x = segments.at(thisSeg).getStartPoint().x;
                if ( min_y > segments.at(thisSeg).getStartPoint().y)
                    min_y = segments.at(thisSeg).getStartPoint().y;
                if ( max_y < segments.at(thisSeg).getEndPoint().y)
                    max_y = segments.at(thisSeg).getEndPoint().y;



                //if there is a seg above AND 'close enough', then qUnprocessed->push()
                if ( thisSeg > 0 &&
                     !isSegUsed[thisSeg-1] &&
                     segments.at(thisSeg).getStartPoint().x == segments.at(thisSeg-1).getStartPoint().x &&
                     segments.at(thisSeg).getStartPoint().y - segments.at(thisSeg-1).getEndPoint().y < VERT_JOIN_LIMIT)
                {
                    //qDebug() << "Up   Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thisSeg-1)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg-1).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thisSeg-1).getStartPoint().x << "," << segments.at(thisSeg-1).getStartPoint().y << ")-("<< segments.at(thisSeg-1).getEndPoint().x << "," << segments.at(thisSeg-1).getEndPoint().y << ")";
                    qUnprocessed.push(thisSeg-1);
                    //take away from pool of raw segs
                    isSegUsed[thisSeg-1] = true;
                    rawSegsLeft--;
                }

                //if there is a seg below AND 'close enough', then qUnprocessed->push()
                if ( thisSeg+1 < segments.size() &&
                     !isSegUsed[thisSeg+1] &&
                     segments.at(thisSeg).getStartPoint().x == segments.at(thisSeg+1).getStartPoint().x &&
                     segments.at(thisSeg+1).getStartPoint().y - segments.at(thisSeg).getEndPoint().y < VERT_JOIN_LIMIT)
                {
                    //qDebug() << "Down Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thisSeg+1)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg+1).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thisSeg+1).getStartPoint().x << "," << segments.at(thisSeg+1).getStartPoint().y << ")-("<< segments.at(thisSeg+1).getEndPoint().x << "," << segments.at(thisSeg+1).getEndPoint().y << ")";
                    qUnprocessed.push(thisSeg+1);
                    //take away from pool of raw segs
                    isSegUsed[thisSeg+1] = true;
                    rawSegsLeft--;
                }

                //! For each segment being processed in a candidate to the RIGHT attempt to join segments within range O(M)
                //if there is a seg overlapping on the right AND 'close enough', then qUnprocessed->push()
                for (int thatSeg = thisSeg + 1; thatSeg < segments.size(); thatSeg++)
                {
                    if ( segments.at(thatSeg).getStartPoint().x - segments.at(thisSeg).getStartPoint().x <=  spacing*HORZ_JOIN_LIMIT)
                    {
                        if ( segments.at(thatSeg).getStartPoint().x > segments.at(thisSeg).getStartPoint().x &&
                             !isSegUsed[thatSeg])
                        {
                            //NOT in same column as thisSeg and is to the right
                            if ( segments.at(thatSeg).getStartPoint().y <= segments.at(thisSeg).getEndPoint().y &&
                                 segments.at(thisSeg).getStartPoint().y <= segments.at(thatSeg).getEndPoint().y)
                            {
                                //thisSeg overlaps with thatSeg
                                //qDebug() <<  "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << "]::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << "]";
                                //! Find intercept O(K), K is number of field border points
                                int intercept = findInterceptFromPerspectiveFrustum(fieldBorders,
                                                                                    segments.at(thisSeg).getStartPoint().x,
                                                                                    segments.at(thatSeg).getStartPoint().x,
                                                                                    spacing*HORZ_JOIN_LIMIT);
                                if ( intercept >= 0 &&
                                     segments.at(thatSeg).getEndPoint().y >= intercept &&
                                     intercept <= segments.at(thisSeg).getEndPoint().y)
                                {
                                    //within HORZ_JOIN_LIMIT
                                    //qDebug() << "Right Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")";
                                    qUnprocessed.push(thatSeg);
                                    //take away from pool of raw segs
                                    isSegUsed[thatSeg] = true;
                                    rawSegsLeft--;

                                }
                                else
                                {
                                    //qDebug() << "|" << float(segments.at(thatSeg).getEndPoint().y) <<  "thatSeg End(y)>= intercept" << intercept << "|";
                                    //qDebug() << "|" << int(intercept) << "intercept <= thisSeg End(y)" << segments.at(thisSeg).getEndPoint().y << "|";
                                }
                            }
                        }
                    }
                    else
                    {
                        thatSeg = segments.size();
                    }
                }

                //! For each segment being processed in a candidate to the LEFT attempt to join segments within range O(M)
                //if there is a seg overlapping on the left AND 'close enough', then qUnprocessed->push()
                for (int thatSeg = thisSeg - 1; thatSeg >= 0; thatSeg--)
                {
                    if ( segments.at(thisSeg).getStartPoint().x - segments.at(thatSeg).getStartPoint().x <=  spacing*HORZ_JOIN_LIMIT)
                    {

                        if ( !isSegUsed[thatSeg] &&
                             segments.at(thatSeg).getStartPoint().x < segments.at(thisSeg).getStartPoint().x)
                        {
                            //NOT in same column as thisSeg and is to the right
                            if ( segments.at(thatSeg).getStartPoint().y <= segments.at(thisSeg).getEndPoint().y &&
                                 segments.at(thisSeg).getStartPoint().y <= segments.at(thatSeg).getEndPoint().y)
                            {
                                //thisSeg overlaps with thatSeg
                                //qDebug() <<  "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << "]::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << "]";
                                //! Find intercept O(K), K is number of field border points
                                int intercept = findInterceptFromPerspectiveFrustum(fieldBorders,
                                                                                    segments.at(thisSeg).getStartPoint().x,
                                                                                    segments.at(thatSeg).getStartPoint().x,
                                                                                    spacing*HORZ_JOIN_LIMIT);

                                if ( intercept >= 0 &&
                                     segments.at(thatSeg).getEndPoint().y >= intercept &&
                                     intercept <= segments.at(thisSeg).getEndPoint().y)
                                {
                                    //within HORZ_JOIN_LIMIT
                                    //qDebug() << "Left Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")";
                                    qUnprocessed.push(thatSeg);
                                    //take away from pool of raw segs
                                    isSegUsed[thatSeg] = true;
                                    rawSegsLeft--;
                                }
                                else
                                {
                                    //qDebug() << "|" << float(segments.at(thatSeg).getEndPoint().y) <<  "thatSeg End(y)>= intercept" << intercept << "|";
                                    //qDebug() << "|" << int(intercept) << "intercept <= thisSeg End(y)" << segments.at(thisSeg).getEndPoint().y << "|";
                                }
                            }
                        }
                    }
                    else
                    {
                        thatSeg = -1;
                    }
                }

                //add thisSeg to CandidateVector
            }//while (!qUnprocessed->empty())
            //qDebug() << "Candidate ready...";
            //HEURISTICS FOR ADDING THIS CANDIDATE AS A ROBOT CANDIDATE
            if ( max_x - min_x > 0 &&                                               // width  is non-zero
                 max_y - min_y > 0 &&                                               // height is non-zero
                 (float)(max_x - min_x) / (float)(max_y - min_y) <= max_aspect &&    // Less    than specified landscape aspect
                 (float)(max_x - min_x) / (float)(max_y - min_y) >= min_aspect &&    // greater than specified portrait aspect
                 segCount >= min_segments                                    // greater than minimum amount of segments to remove noise
                 )
            {
                //qDebug() << "CANDIDATE FINISHED::" << segCount << " segments, aspect:" << ( (float)(max_x - min_x) / (float)(max_y - min_y)) << ", Coords:(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << "), width: " << (max_x - min_x) << ", height: " << (max_y - min_y);
                int max_col = 0;
                for (int i = 0; i < validColours.size(); i++)
                {
                    if (i != max_col && colourHistogram[i] > colourHistogram[max_col])
                        max_col = i;
                }
                ObjectCandidate temp(min_x, min_y, max_x, max_y, validColours.at(max_col));
                candidateList.push_back(temp);
            }

        }//while(rawSegsLeft)

    }//if (!segments.empty())
    return candidateList;
}

std::vector<ObjectCandidate> Vision::classifyCandidatesDBSCAN(std::vector< TransitionSegment > segments,
                                        std::vector<Vector2<int> >&fieldBorders,
                                        std::vector<unsigned char> validColours,
                                        int spacing,
                                        float min_aspect, float max_aspect, int min_segments)
{
    std::vector<ObjectCandidate> candidateList;



    return candidateList;
}

bool Vision::isValidColour(unsigned char colour, std::vector<unsigned char> colourList)
{
    bool result = false;
    if (colourList.size())
    {
        std::vector<unsigned char>::const_iterator nextCol = colourList.begin();
        for (;nextCol != colourList.end(); nextCol++)
        {
            if (colour == *(nextCol.base()))
            {
                result = true;
                break;
            }
        }
    }
    return result;
}

int Vision::findYFromX(std::vector<Vector2<int> >&points, int x)
{

    int y = 0;
    int left_x = -1;
    int left_y = 0;
    int right_x = -1;
    int right_y = 0;
    std::vector< Vector2<int> >::const_iterator nextPoint = points.begin();

    for(; nextPoint != points.end(); nextPoint++)
    {
        if (nextPoint->x == x)
        {
            return nextPoint->y;
        }

        if (left_x < nextPoint->x && nextPoint->x < x)
        {
            left_x = nextPoint->x;
            left_y = nextPoint->y;
        }

        if ( (right_x > nextPoint->x || right_x < 0) && nextPoint->x > x)
        {
            right_x = nextPoint->x;
            right_y = nextPoint->y;
        }

    }
    //qDebug() << "findYFromX" << y;
    if (right_x - left_x > 0)
        y = left_y + (right_y-left_y) * (x-left_x) / (right_x-left_x);
    //qDebug() << "findYFromX" << y;
    return y;
}

int Vision::findInterceptFromPerspectiveFrustum(std::vector<Vector2<int> >&points, int current_x, int target_x, int spacing)
{
    int intercept = 0;
    if (current_x == target_x)
    {
        //qDebug() << "Intercept -1 =";
        return -1;
    }

    int y = findYFromX(points, current_x);
    int diff_x;
    int diff_y = currentImage->height() - y;

    if (current_x < target_x)
    {
        diff_x = target_x - current_x;
    }

    if (target_x < current_x)
    {
        diff_x = current_x - target_x;
    }

    if (diff_x > spacing)
    {
        intercept = currentImage->height();
    }
    else if ( diff_x > spacing/2)
    {
        intercept = y + diff_y/2;
    }
    else if ( diff_x > spacing/4)
    {
        intercept = y + diff_y/4;
    }
    else
    {
        intercept = y;
    }

    //qDebug() << "findInterceptFromPerspectiveFrustum intercept:"<<intercept<<" {y:"<< y << ", height:" << currentImage->height() << ", spacing:" << spacing << ", target_x:" << target_x << ", current_x:" << current_x << "}";
    if (intercept > currentImage->height())
        intercept = -2;
    return intercept;
}

bool Vision::sortTransitionSegments(TransitionSegment a, TransitionSegment b)
{
    return (a.getStartPoint().x < b.getStartPoint().x || (a.getStartPoint().x == b.getStartPoint().x && a.getEndPoint().y <= b.getStartPoint().y));
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

std::vector<LSFittedLine> Vision::DetectLines(ClassifiedSection* scanArea,int spacing)
{
    LineDetection* LineDetector = new LineDetection();
    int image_width = currentImage->width();
    int image_height = currentImage->height();
    LineDetector->FormLines(scanArea,image_width,image_height,spacing);
    qDebug() << "Lines Detection completed";
    std::vector<CornerPoint> cornerPoints= LineDetector->cornerPoints;
    std::vector<LSFittedLine> fieldLines= LineDetector->fieldLines;

    return fieldLines;
}
