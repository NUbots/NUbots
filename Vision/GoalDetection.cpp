#include "GoalDetection.h"
#include "ClassificationColours.h"
#include "TransitionSegment.h"
#include "ScanLine.h"
#include "ClassifiedSection.h"
//#include <QDebug>
#include "debug.h"
#include "Tools/Math/General.h"
using namespace mathGeneral;


GoalDetection::GoalDetection()
{
    //debug<< "Vision::GoalDetection : GoalDetection Class created" << endl;
}
GoalDetection::~GoalDetection()
{
}

//! Finds the ball segments and groups updates the goal in fieldObjects (Vision is used to further classify the object)
ObjectCandidate GoalDetection::FindGoal(std::vector <ObjectCandidate>& FO_Candidates,std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,FieldObjects* AllObjects, std::vector< TransitionSegment > horizontalSegments,Vision* vision,int height,int width)
{
	ObjectCandidate result;


        vector < ObjectCandidate > ::iterator it;

        //! Go through all the candidates: to find a possible goal
        for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); )
	{
            if(!isObjectAPossibleGoal(*it))
            {
                //qDebug() << "Erasing FO_CANDIDATE";
                it = FO_Candidates.erase(it);
                continue;
            }
            //qDebug() << "Crash Check: Before Extend with Horizontal Segments Detection:";
            ExtendGoalAboveHorizon(&(*it), FO_AboveHorizonCandidates,horizontalSegments);
            ++it;
        }
        //ADD REMAINING GOAL CANDIDATES above horizon:
        FO_Candidates.insert(FO_Candidates.end(), FO_AboveHorizonCandidates.begin(), FO_AboveHorizonCandidates.end());
        FO_AboveHorizonCandidates.clear();

        //Combine Any "OverLapping Candidates:
        CombineOverlappingCandidates(FO_Candidates);

        for (it = FO_Candidates.begin(); it  < FO_Candidates.end(); )
        {
        //qDebug() << "Crash Check: Before Closely classify Detection:";
            classifyGoalClosely(&(*it), vision, height, width);
            //qDebug() << "Crash Check: Before Ratio check Detection:";
            bool RatioOK = isCorrectCheckRatio(*it, height, width);
            if(RatioOK)
            {
                float FinalDistance;
                FinalDistance = FindGoalDistance(*it,vision);
                ++it;
                //debug << "Distance to Goal[" << 0 <<"]: "<< FinalDistance << endl;
            }
            else
            {
                it = FO_Candidates.erase(it);
            }
	}

        return result;
}

bool GoalDetection::isObjectAPossibleGoal(ObjectCandidate PossibleGoal)
{
    if( PossibleGoal.getColour() == 	ClassIndex::blue ||
        PossibleGoal.getColour() == 	ClassIndex::shadow_blue ||
        PossibleGoal.getColour() == 	ClassIndex::yellow ||
        PossibleGoal.getColour() == 	ClassIndex::yellow_orange)
    {
        return true;
    }
    else{
        return false;
    }
}
void GoalDetection::ExtendGoalAboveHorizon(ObjectCandidate* PossibleGoal,
                                           std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
                                           std::vector < TransitionSegment > horizontalSegments)
{
    Vector2<int> TopLeft = PossibleGoal->getTopLeft();
    Vector2<int> BottomRight = PossibleGoal->getBottomRight();
    int Colour = PossibleGoal->getColour();
    int min = TopLeft.x;
    int max = BottomRight.x;
    int margin = 16;
    if((int)horizontalSegments.size() ==0 && (int)FO_AboveHorizonCandidates.size() ==0) return;

    vector < ObjectCandidate > ::iterator itAboveHorizon;
    debug << "AboveHoriCands:" << endl;
    for (itAboveHorizon = FO_AboveHorizonCandidates.begin(); itAboveHorizon < FO_AboveHorizonCandidates.end(); )
    {
        if( itAboveHorizon->getTopLeft().x > TopLeft.x - margin &&
            itAboveHorizon->getBottomRight().x < BottomRight.x + margin &&
            Colour == itAboveHorizon->getColour())
        {
            if(itAboveHorizon->getTopLeft().x < TopLeft.x)
            {
                TopLeft.x = itAboveHorizon->getTopLeft().x;
            }
            if(itAboveHorizon->getTopLeft().y > TopLeft.y)
            {
                TopLeft.y = itAboveHorizon->getTopLeft().y;
            }
            if(itAboveHorizon->getBottomRight().x < BottomRight.x)
            {
                BottomRight.x = itAboveHorizon->getBottomRight().x;
            }
            if(itAboveHorizon->getBottomRight().y > BottomRight.y)
            {
                BottomRight.y = itAboveHorizon->getBottomRight().y;
            }
            PossibleGoal->setTopLeft(TopLeft);
            PossibleGoal->setBottomRight(BottomRight);
            itAboveHorizon = FO_AboveHorizonCandidates.erase(itAboveHorizon);
            debug << "OverLapping: Join Cand " << endl;
            //usedAbovehorizonCandidate[i] = true;
            //qDebug() <<"Found OverLapping Candidate Above horizon" << FO_AboveHorizonCandidates.size();
        }
        else
        {
            ++itAboveHorizon;
        }

    }
    //SCANS UP THE IMAGE
    std::vector<TransitionSegment>::reverse_iterator revIt = horizontalSegments.rbegin();
    for (; revIt != horizontalSegments.rend(); ++revIt)
    {
        //qDebug() << "Crash Check: Access HZsegs: " << i;
        TransitionSegment tempSegment = *revIt;
        //! If     Candidate colour is a blue varient and Segment colour is a blue varient OR
        //!        Candidate colour is a yellow varient and Segment colour is a yellow varient
        //! THEN   Extend the Candidate with Segment information.

        if(     ((Colour == ClassIndex::blue || Colour == ClassIndex::shadow_blue) &&
                (tempSegment.getColour() == ClassIndex::blue || tempSegment.getColour() == ClassIndex::shadow_blue)) ||
                ((Colour == ClassIndex::yellow || Colour == ClassIndex::yellow_orange) &&
                (tempSegment.getColour() == ClassIndex::yellow || tempSegment.getColour() == ClassIndex::yellow_orange))
            )
        {
            if(tempSegment.getStartPoint().x > min-margin && tempSegment.getEndPoint().x < max+margin)
            {
                //qDebug() << "Found Segment at " << tempSegment.getStartPoint().x << "," << tempSegment.getStartPoint().y;
                if(tempSegment.getStartPoint().y < PossibleGoal->getTopLeft().y)
                {
                    Vector2<int> tempPoint;
                    tempPoint.x = PossibleGoal->getTopLeft().x;
                    tempPoint.y = tempSegment.getStartPoint().y;
                    PossibleGoal->setTopLeft(tempPoint);
                }
                if(tempSegment.getStartPoint().x < PossibleGoal->getTopLeft().x)
                {
                    Vector2<int> tempPoint;
                    tempPoint.x = tempSegment.getStartPoint().x;
                    tempPoint.y = PossibleGoal->getTopLeft().y;
                    PossibleGoal->setTopLeft(tempPoint);
                }
                if(tempSegment.getEndPoint().y > PossibleGoal->getBottomRight().y)
                {
                    Vector2<int> tempPoint;
                    tempPoint.x = PossibleGoal->getBottomRight().x;
                    tempPoint.y = tempSegment.getEndPoint().y;
                    PossibleGoal->setBottomRight(tempPoint);
                }
                if(tempSegment.getEndPoint().x > PossibleGoal->getBottomRight().x)
                {
                    Vector2<int> tempPoint;
                    tempPoint.x = tempSegment.getEndPoint().x;
                    tempPoint.y = PossibleGoal->getBottomRight().y;
                    PossibleGoal->setBottomRight(tempPoint);
                }
            }
        }
    }
}


void GoalDetection::classifyGoalClosely(ObjectCandidate* PossibleGoal,Vision* vision,int height, int width)
{
    Vector2<int> TopLeft = PossibleGoal->getTopLeft();
    Vector2<int> BottomRight = PossibleGoal->getBottomRight();
    int y =  (int)TopLeft.y;
    Vector2<int> SegStart;
    SegStart.x = TopLeft.x;
    SegStart.y = y;
    Vector2<int> SegEnd;
    SegEnd.x = BottomRight.x;
    SegEnd.y = y;
    TransitionSegment tempSeg(SegStart,SegEnd,ClassIndex::unclassified,PossibleGoal->getColour(),ClassIndex::unclassified);
    //qDebug() << "segments (start): " << tempSeg->getStartPoint().x << "," << tempSeg->getStartPoint().y;
    ScanLine tempLine;

    int spacings = 8;
    int direction = ClassifiedSection::RIGHT;
    vision->CloselyClassifyScanline(&tempLine,&tempSeg,spacings, direction);

    //qDebug() << "segments found: " << tempLine->getNumberOfSegments();
    //! Debug Output for small scans:
    int min = PossibleGoal->getTopLeft().y;
    for(int i = 0; i < tempLine.getNumberOfSegments(); i++)
    {
        TransitionSegment* tempSegment = tempLine.getSegment(i);
        //qDebug() << "segments (start): " << tempSeg->getStartPoint().x << "," << tempSeg->getStartPoint().y;
        //qDebug() << "segments (end): " << tempSeg->getEndPoint().x << "," << tempSeg->getEndPoint().y;
        if(tempSegment->getStartPoint().y < min)
        {
            min = tempSegment->getStartPoint().y;
        }
    }
    Vector2<int> tempTopLeft;
    tempTopLeft.x = PossibleGoal->getTopLeft().x;
    tempTopLeft.y = min;
    PossibleGoal->setTopLeft(tempTopLeft);
    //qDebug() << "Extending Top Of Goal: " << tempTopLeft.x << "," << tempTopLeft.y;
    return;

}

void GoalDetection::CombineOverlappingCandidates(std::vector <ObjectCandidate>& FO_Candidates)
{
    vector < ObjectCandidate > ::iterator it;
    int boarder = 10;
    //! Go through all the candidates: to find overlapping and add to the bigger object candidate:
    for(it = FO_Candidates.begin(); it  < FO_Candidates.end();  it++)
    {
        vector < ObjectCandidate > ::iterator itInside;
        for(itInside = it+1; itInside  < FO_Candidates.end(); itInside++)
        {
            //! CHECK INSIDE Object TOPLEFT is within outside Object
            if((    it->getTopLeft().x-boarder      <= itInside->getTopLeft().x     &&
                    it->getTopLeft().y-boarder      <= itInside->getTopLeft().y)    &&
               (    it->getBottomRight().x+boarder  >= itInside->getTopLeft().x     &&
                    it->getBottomRight().y+boarder  >= itInside->getTopLeft().y)  )
            {
                //EDIT OUTSIDE
                Vector2<int> tempTopLeft,tempBottomRight;
                tempTopLeft = it->getTopLeft();
                tempBottomRight = it->getBottomRight();
                //UPDATECODE:
                if(tempTopLeft.x > itInside->getTopLeft().x)
                {
                    tempTopLeft.x = itInside->getTopLeft().x;
                }
                if(tempTopLeft.y > itInside->getTopLeft().y)
                {
                    tempTopLeft.y = itInside->getTopLeft().y;
                }
                if(tempBottomRight.x < itInside->getBottomRight().x)
                {
                    tempBottomRight.x = itInside->getBottomRight().x;
                }
                if(tempBottomRight.y < itInside->getBottomRight().y)
                {
                    tempBottomRight.y = itInside->getBottomRight().y;
                }
                it->setTopLeft(tempTopLeft);
                it->setBottomRight(tempBottomRight);
                FO_Candidates.erase(itInside);
                itInside = FO_Candidates.begin();
                //qDebug() << "Found: Overlapping TopLeft Goal";
            }
            //! CHECK INSIDE Object BOTTOMRIGHT is within outside Object
            else if ((  it->getTopLeft().x - boarder     <= itInside->getBottomRight().x     &&
                        it->getTopLeft().y - boarder     <= itInside->getBottomRight().y)    &&
                     (  it->getBottomRight().x + boarder >= itInside->getBottomRight().x     &&
                        it->getBottomRight().y + boarder >= itInside->getBottomRight().y)  )
            {
                //EDIT OUTSIDE
                Vector2<int> tempTopLeft,tempBottomRight;
                tempTopLeft = it->getTopLeft();
                tempBottomRight = it->getBottomRight();
                //UPDATECODE:
                if(tempTopLeft.x > itInside->getTopLeft().x)
                {
                    tempTopLeft.x = itInside->getTopLeft().x;
                }
                if(tempTopLeft.y > itInside->getTopLeft().y)
                {
                    tempTopLeft.y = itInside->getTopLeft().y;
                }
                if(tempBottomRight.x < itInside->getBottomRight().x)
                {
                    tempBottomRight.x = itInside->getBottomRight().x;
                }
                if(tempBottomRight.y < itInside->getBottomRight().y)
                {
                    tempBottomRight.y = itInside->getBottomRight().y;
                }
                it->setTopLeft(tempTopLeft);
                it->setBottomRight(tempBottomRight);
                FO_Candidates.erase(itInside);
                itInside = FO_Candidates.begin();
                //qDebug() << "Found: Overlapping BottomRight Goal";
            }
            //! CHECK OUTSIDE Object TOPLEFT is within inside Object
            else if((    itInside->getTopLeft().x-boarder      <= it->getTopLeft().x     &&
                        itInside->getTopLeft().y-boarder      <= it->getTopLeft().y)    &&
                    (   itInside->getBottomRight().x+boarder  >= it->getTopLeft().x     &&
                        itInside->getBottomRight().y+boarder  >= it->getTopLeft().y)  )
            {
                //EDIT OUTSIDE
                Vector2<int> tempTopLeft,tempBottomRight;
                tempTopLeft = it->getTopLeft();
                tempBottomRight = it->getBottomRight();
                //UPDATECODE:
                if(tempTopLeft.x > itInside->getTopLeft().x)
                {
                    tempTopLeft.x = itInside->getTopLeft().x;
                }
                if(tempTopLeft.y > itInside->getTopLeft().y)
                {
                    tempTopLeft.y = itInside->getTopLeft().y;
                }
                if(tempBottomRight.x < itInside->getBottomRight().x)
                {
                    tempBottomRight.x = itInside->getBottomRight().x;
                }
                if(tempBottomRight.y < itInside->getBottomRight().y)
                {
                    tempBottomRight.y = itInside->getBottomRight().y;
                }
                it->setTopLeft(tempTopLeft);
                it->setBottomRight(tempBottomRight);
                FO_Candidates.erase(itInside);
                itInside = FO_Candidates.begin();
                //qDebug() << "Found: Overlapping TopLeft Goal OutSide";
            }
            //! CHECK OUTSIDE Object BOTTOMRIGHT is within inside Object
            else if ((  itInside->getTopLeft().x - boarder     <= it->getBottomRight().x     &&
                        itInside->getTopLeft().y - boarder     <= it->getBottomRight().y)    &&
                     (  itInside->getBottomRight().x + boarder >= it->getBottomRight().x     &&
                        itInside->getBottomRight().y + boarder >= it->getBottomRight().y)  )
            {
                //EDIT OUTSIDE
                Vector2<int> tempTopLeft,tempBottomRight;
                tempTopLeft = it->getTopLeft();
                tempBottomRight = it->getBottomRight();
                //UPDATECODE:
                if(tempTopLeft.x > itInside->getTopLeft().x)
                {
                    tempTopLeft.x = itInside->getTopLeft().x;
                }
                if(tempTopLeft.y > itInside->getTopLeft().y)
                {
                    tempTopLeft.y = itInside->getTopLeft().y;
                }
                if(tempBottomRight.x < itInside->getBottomRight().x)
                {
                    tempBottomRight.x = itInside->getBottomRight().x;
                }
                if(tempBottomRight.y < itInside->getBottomRight().y)
                {
                    tempBottomRight.y = itInside->getBottomRight().y;
                }
                it->setTopLeft(tempTopLeft);
                it->setBottomRight(tempBottomRight);
                FO_Candidates.erase(itInside);
                itInside = FO_Candidates.begin();
                //qDebug() << "Found: Overlapping BottomRight Goal OutSide";
            }
        }
    }
}

bool GoalDetection::isCorrectCheckRatio(ObjectCandidate PossibleGoal,int height, int width)
{
    //qDebug() << "Checking Ratio: " << PossibleBall.aspect();

    //! Check if at Edge of Screen, if so continue with other checks, otherwise, look at ratio and check if in thresshold
    int boarder = 5; //! Boarder of pixels
    if (PossibleGoal.getBottomRight().x <= width-boarder &&
        PossibleGoal.getBottomRight().y <= height-boarder &&
        PossibleGoal.getTopLeft().x >=0+boarder &&
        PossibleGoal.getTopLeft().y >=0+boarder  )
    {
        //POSSIBLE GOALS ARE:
        //      Objects which have grouped segments,
        //      or objects with one segment, but very small (still like to consider).
        if((PossibleGoal.aspect() > 0 && PossibleGoal.aspect() < 0.3 )|| PossibleGoal.aspect()==0)
        {

            return true;
        }
        else
        {
            //qDebug() << "Thrown out due to incorrect ratio";
            return false;
        }
    }
    else
    {
        //qDebug() << "Returned True at edge of screen";
        return true;
    }
}
float GoalDetection::FindGoalDistance(ObjectCandidate PossibleGoal, Vision* vision)
{
    float distance = 0.0;
    std::vector < TransitionSegment > tempSegments = PossibleGoal.getSegments();
    std::vector < Vector2<int> > midpoints, leftPoints, rightPoints;
    Vector2<int> tempStart, tempEnd;


    // Joins segments on same scanline and finds MIDPOINTS, leftPoints and rightPoints:
    for (int i =0; i< (int)tempSegments.size(); i++)
    {\
        tempStart = tempSegments[i].getStartPoint();
        tempEnd = tempSegments[i].getEndPoint();
        //qDebug() << i<<": " <<tempSegments[i].getStartPoint().x << "," << tempSegments[i].getStartPoint().y
        //                    << tempSegments[i].getEndPoint().x  << "," << tempSegments[i].getEndPoint().y  ;

        int j = i+1;
        if(j < (int)tempSegments.size())
        {
            while(tempStart.y == tempSegments[j].getStartPoint().y)
            {
                tempEnd = tempSegments[j].getEndPoint();
                j++;
                if(j > (int)tempSegments.size())
                {
                    break;
                }
            }
        }

        i = j-1;
        if(tempEnd.x-tempStart.x > 10)
        {
            Vector2<int> tempMidPoint;
            tempMidPoint.x = (int)((tempEnd.x -tempStart.x)/2)+tempStart.x;
            tempMidPoint.y = (int)((tempEnd.y - tempStart.y)/2)+tempStart.y;
            midpoints.push_back(tempMidPoint);
            leftPoints.push_back(tempStart);
            rightPoints.push_back(tempEnd);
        }

    }
    //qDebug() << "Number Of MidPoints: " <<(int) midpoints.size();
    if(midpoints.size() < 3 )
    {
        float FinalDistance;
        float GoalHeightDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (PossibleGoal.getBottomRight().y - PossibleGoal.getTopLeft().y); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
        float GoalWidthDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ ((PossibleGoal.getBottomRight().x - PossibleGoal.getTopLeft().x)*8); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS

        if(GoalHeightDistance > GoalWidthDistance)
        {
            FinalDistance = GoalWidthDistance;
            //debug <<"WIDTH GOAL Distance: " << GoalWidthDistance <<endl;
        }
        else
        {
            FinalDistance = GoalHeightDistance;
            //debug <<"Height GOAL Distance: " << GoalHeightDistance <<endl;
        }
        return FinalDistance;
    }

    //FORM EQUATION if MidPointLine
    //debug << "Number Of MidPoints: " <<(int) midpoints.size() << endl;
    LSFittedLine midPointLine;
    for (int i = 0; i < (int) midpoints.size(); i++)
    {

        LinePoint  point;
        point.x = midpoints[i].x;
        point.y = midpoints[i].y;
        midPointLine.addPoint(point);
        //debug << "MidPoint: \t" << point.x << "," <<point.y << endl;
    }
    //debug << "Equation of Line is: " << midPointLine.getA()<< "x + " <<  midPointLine.getB() << "y + " << midPointLine.getC() << " = 0" << endl;
    //debug << "Interescting Screen at TOP: \t"<< midPointLine.findXFromY(0)<< ","<< 0 << endl;
    //debug << "Interescting Screen at TOP: \t"<< midPointLine.findXFromY(320)<< ","<< 320 << endl;
    float leftWidth = 0;
    float rightWidth = 0;
    for(int i = 0 ; i < (int)leftPoints.size(); i++)
    {
        Vector2<int> point = leftPoints[i];
        leftWidth = leftWidth + DistanceLineToPoint(midPointLine, point);
    }
    leftWidth = leftWidth / (float)leftPoints.size();
    //debug << "Distance from centre of Line to Left Points: " << leftWidth<< endl;
    for(int i = 0 ; i < (int)rightPoints.size(); i++)
    {
        Vector2<int> point = rightPoints[i];
        rightWidth = rightWidth + DistanceLineToPoint(midPointLine, point);
    }
    rightWidth = rightWidth / (float)rightPoints.size();
    //debug << "Distance from centre of Line to Right Points: " << rightWidth << endl;
    float totalwidth = rightWidth + leftWidth;
    distance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ ((totalwidth)*8); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
    return distance;
}

float GoalDetection::DistanceLineToPoint(LSFittedLine midPointLine, Vector2<int>  point)
{
    float distance = fabs( - point.x * midPointLine.getA() + point.y *  midPointLine.getB() + midPointLine.getC())
                   / sqrt( midPointLine.getA() *  midPointLine.getA() + midPointLine.getB() *  midPointLine.getB());
    //debug << "Distance L2P: " << distance << "\t Using: "<<point.x << ","<< point.y<<endl;
    return distance;
}
