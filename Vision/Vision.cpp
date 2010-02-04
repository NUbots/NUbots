/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
  @author Steven Nicklin
*/

#include "Vision.h"
#include "Tools/Image/NUImage.h"
#include "ClassificationColours.h"
#include "LineDetection.h"
#include <QDebug>
#include <boost/circular_buffer.hpp>
#include <queue>

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
    unsigned char currentColour = 0; //!< Colour in the current segment
    //! initialising circular buffer
    int bufferSize = 2;
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
                    qDebug() << "End Of Line Detected: " << currentPoint.x << ","<< currentPoint.y;
                }
                tempStartPoint = currentPoint;
                beforeColour = currentColour;
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
                            currentPoint.y = startPoint.y - j;// - bufferSize;
                        }
                        else if(direction == ClassifiedSection::LEFT)
                        {
                            currentPoint.x = startPoint.x - j;// - bufferSize;
                            currentPoint.y = startPoint.y;
                        }
                        tempTransition = new TransitionSegment(tempStartPoint, currentPoint, beforeColour, currentColour, afterColour);
                        tempLine->addSegement(tempTransition);
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

std::vector<RobotCandidate> Vision::classifyRobotCandidates(std::vector< TransitionSegment > segments)
{
    std::vector<RobotCandidate> candidateList;

    const float MAX_ASPECT = 2.0;
    const float MIN_ASPECT = 0.1;
    const int VERT_JOIN_LIMIT = 3;
    const int HORZ_JOIN_LIMIT = 2;
    const int HORZ_JOIN_SCALING = 4;
    const int SEG_COUNT_THRESHOLD = 12;
    const int COLOUR_SEG_THRESHOLD = 5;

    if (!segments.empty())
    {
        std::queue<int> qUnprocessed;
        unsigned int rawSegsLeft = segments.size();
        int nextRawSeg = 0;
        bool isSegUsed [segments.size()];
        for (unsigned int i = 0; i < segments.size(); i++)
        {
            //may have non-robot colour segments, so pre-mark them as used
            if (isRobotColour(segments.at(i).getColour()))
            {
                //qDebug() << ClassIndex::getColourNameFromIndex(segments.at(i).getColour()) << isRobotColour(segments.at(i).getColour());
                isSegUsed[i] = false;
            }
            else
            {
                rawSegsLeft--;
                isSegUsed[i] = true;
            }

        }
        //qDebug() << "rawSegsLeft: " << rawSegsLeft << "/" << (segments.size()-1);
        while(rawSegsLeft)
        {

            //Roll through and find first unused segment
            nextRawSeg = 0;

            while(isSegUsed[nextRawSeg] && nextRawSeg < segments.size()) nextRawSeg++;
            //Prime unprocessed segment queue to build next candidate
            qUnprocessed.push(nextRawSeg);
            //take away from pool of raw segs
            isSegUsed[nextRawSeg] = true;
            rawSegsLeft--;

            int teamColour = 0;
            int min_x, max_x, min_y, max_y, segCount;
            min_x = segments.at(nextRawSeg).getStartPoint().x;
            max_x = segments.at(nextRawSeg).getStartPoint().x;
            min_y = segments.at(nextRawSeg).getStartPoint().y;
            max_y = segments.at(nextRawSeg).getEndPoint().y;
            segCount = 0;

            //Build candidate
            while (!qUnprocessed.empty())
            {
                unsigned int thisSeg;
                thisSeg = qUnprocessed.front();
                qUnprocessed.pop();
                segCount++;

                if ( segments.at(thisSeg).getColour() == ClassIndex::shadow_blue)
                {
                    teamColour++;
                }

                if ( segments.at(thisSeg).getColour() == ClassIndex::red)
                {
                    teamColour--;
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
                     segments.at(thisSeg).getStartPoint().x == segments.at(thisSeg-1).getStartPoint().x &&
                     -(segments.at(thisSeg-1).getEndPoint().y - segments.at(thisSeg).getStartPoint().y) < VERT_JOIN_LIMIT &&
                     !isSegUsed[thisSeg-1])
                {
                    //qDebug() << "Up   Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thisSeg-1)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg-1).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thisSeg-1).getStartPoint().x << "," << segments.at(thisSeg-1).getStartPoint().y << ")-("<< segments.at(thisSeg-1).getEndPoint().x << "," << segments.at(thisSeg-1).getEndPoint().y << ")";
                    qUnprocessed.push(thisSeg-1);
                    //take away from pool of raw segs
                    isSegUsed[thisSeg-1] = true;
                    rawSegsLeft--;
                }
                //if there is a seg below AND 'close enough', then qUnprocessed->push()
                if ( thisSeg+1 < segments.size() &&
                     segments.at(thisSeg).getStartPoint().x == segments.at(thisSeg+1).getStartPoint().x &&
                     -(segments.at(thisSeg).getEndPoint().y - segments.at(thisSeg+1).getStartPoint().y) < VERT_JOIN_LIMIT &&
                     !isSegUsed[thisSeg+1])
                {
                    //qDebug() << "Down Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thisSeg+1)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg+1).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thisSeg+1).getStartPoint().x << "," << segments.at(thisSeg+1).getStartPoint().y << ")-("<< segments.at(thisSeg+1).getEndPoint().x << "," << segments.at(thisSeg+1).getEndPoint().y << ")";
                    qUnprocessed.push(thisSeg+1);
                    //take away from pool of raw segs
                    isSegUsed[thisSeg+1] = true;
                    rawSegsLeft--;
                }
                //if there is a seg overlapping on the right AND 'close enough', then qUnprocessed->push()
                for (int thatSeg = thisSeg + 1; thatSeg < segments.size(); thatSeg++)
                {
                    if ( segments.at(thatSeg).getStartPoint().x > segments.at(thisSeg).getStartPoint().x &&
                         !isSegUsed[thatSeg])
                    {
                        //NOT in same column as thisSeg and is to the right
                        if ( segments.at(thatSeg).getStartPoint().y < segments.at(thisSeg).getEndPoint().y &&
                             segments.at(thisSeg).getStartPoint().y < segments.at(thatSeg).getEndPoint().y)
                        {
                            //thisSeg overlaps with thatSeg
                            if ( segments.at(thatSeg).getStartPoint().x - segments.at(thisSeg).getStartPoint().x < HORZ_JOIN_LIMIT * HORZ_JOIN_SCALING )
                            {
                                //within HORZ_JOIN_LIMIT
                                //qDebug() << "Right Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")";
                                qUnprocessed.push(thatSeg);
                                //take away from pool of raw segs
                                isSegUsed[thatSeg] = true;
                                rawSegsLeft--;
                                thatSeg = segments.size();
                            }
                        }
                    }
                }
                //if there is a seg overlapping on the left AND 'close enough', then qUnprocessed->push()
                for (int thatSeg = thisSeg - 1; thatSeg >= 0; thatSeg--)
                {
                    if ( !isSegUsed[thatSeg] &&
                         segments.at(thatSeg).getStartPoint().x < segments.at(thisSeg).getStartPoint().x)
                    {
                        //NOT in same column as thisSeg and is to the right
                        if ( segments.at(thatSeg).getStartPoint().y < segments.at(thisSeg).getEndPoint().y &&
                             segments.at(thisSeg).getStartPoint().y < segments.at(thatSeg).getEndPoint().y)
                        {
                            //thisSeg overlaps with thatSeg
                            if ( segments.at(thisSeg).getStartPoint().x - segments.at(thatSeg).getStartPoint().x < HORZ_JOIN_LIMIT * HORZ_JOIN_SCALING )
                            {
                                //within HORZ_JOIN_LIMIT
                                //qDebug() << "Left Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")";
                                qUnprocessed.push(thatSeg);
                                //take away from pool of raw segs
                                isSegUsed[thatSeg] = true;
                                rawSegsLeft--;
                                thatSeg = -1;
                            }
                        }
                    }
                }

                //add thisSeg to CandidateVector
            }//while (!qUnprocessed->empty())
            if ( max_x - min_x > 0 &&
                 max_y - min_y > 0 &&
                 (float)(max_x - min_x) / (float)(max_y - min_y) < MAX_ASPECT &&
                 (float)(max_x - min_x) / (float)(max_y - min_y) > MIN_ASPECT &&
                 segCount > SEG_COUNT_THRESHOLD)
            {
                //qDebug() << "CANDIDATE FINISHED::" << segCount << " segments, aspect:" << ( (float)(max_x - min_x) / (float)(max_y - min_y)) << ", Coords:(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << "), width: " << (max_x - min_x) << ", height: " << (max_y - min_y) << ", dist from bottom: " << (120 - max_y) ;
                if (teamColour >= COLOUR_SEG_THRESHOLD)
                {
                    RobotCandidate temp(min_x, min_y, max_x, max_y, ClassIndex::shadow_blue);
                    candidateList.push_back(temp);
                }
                else if (teamColour <= -COLOUR_SEG_THRESHOLD)
                {
                    RobotCandidate temp(min_x, min_y, max_x, max_y, ClassIndex::red);
                    candidateList.push_back(temp);
                }
                else
                {
                    RobotCandidate temp(min_x, min_y, max_x, max_y);
                    candidateList.push_back(temp);
                }


            }



        }//while(rawSegsLeft)

    }//if (!segments.empty())
    return candidateList;
}

bool Vision::isRobotColour(unsigned char colour)
{
    return ( colour == ClassIndex::white ||
             colour == ClassIndex::red   ||
             colour == ClassIndex::shadow_blue);
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

void Vision::DetectLines(ClassifiedSection* scanArea)
{
    LineDetection* LineDetector = new LineDetection();
    int image_width = currentImage->width();
    LineDetector->FormLines(scanArea,image_width);

    return;
}
