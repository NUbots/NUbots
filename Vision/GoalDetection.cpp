#include "GoalDetection.h"
#include "ClassificationColours.h"
#include "TransitionSegment.h"
#include "ScanLine.h"
#include "ClassifiedSection.h"
#include <QDebug>
#include "debug.h"

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
        bool usedAboveHorizonCandidate[FO_AboveHorizonCandidates.size()];
        for(int i = 0; i < FO_AboveHorizonCandidates.size();i++)
        {
            usedAboveHorizonCandidate[i] = false;
        }
        //! Go through all the candidates: to find a possible goal
	for(unsigned int i = 0; i  < FO_Candidates.size(); i++)
	{
            if(!isObjectAPossibleGoal(FO_Candidates[i]))
            {
                continue;
            }
            //qDebug() << "Crash Check: Before Extend with Horizontal Segments Detection:";
            ExtendGoalAboveHorizon(&FO_Candidates[i], FO_AboveHorizonCandidates, usedAboveHorizonCandidate, horizontalSegments);
        }
        //ADD REMAINING GOAL CANDIDATES above horizon:
        for( int i = 0; i < (int)FO_AboveHorizonCandidates.size(); i++)
        {
            if(!usedAboveHorizonCandidate[i])
            {
                FO_Candidates.push_back(FO_AboveHorizonCandidates[i]);
            }
        }

        for (int i = 0; i < FO_Candidates.size(); i++)
        {
        //qDebug() << "Crash Check: Before Closely classify Detection:";
            classifyGoalClosely(&FO_Candidates[i], vision, height, width);
            //qDebug() << "Crash Check: Before Ratio check Detection:";
            bool RatioOK = isCorrectCheckRatio(FO_Candidates[i], height, width);
            //qDebug() << "RatioOK: " << RatioOK;
            int boarder = 16;
            /*
            if ( RatioOK && FO_Candidates[i].getBottomRight().x <= width-boarder &&
                        FO_Candidates[i].getBottomRight().y <= height-boarder &&
                        FO_Candidates[i].getTopLeft().x >=0+boarder &&
                        FO_Candidates[i].getTopLeft().y >=0+boarder  )
            {
                //use height

                qDebug() << i <<": HEIGHT GOAL Distance: " << GoalDistance;
            }
            else if(RatioOK)
            {

                //use width
                //qDebug() << vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS();
                float GoalDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ ((FO_Candidates[i].getBottomRight().x - FO_Candidates[i].getTopLeft().x)*8); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
                qDebug() << i<<": WIDTH GOAL Distance: " << GoalDistance;
            }
            */
            if(RatioOK)
            {
                float FinalDistance;
                //float GoalHeightDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (FO_Candidates[i].getBottomRight().y - FO_Candidates[i].getTopLeft().y); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
                //float GoalWidthDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ ((FO_Candidates[i].getBottomRight().x - FO_Candidates[i].getTopLeft().x)*8); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
                FinalDistance = FindGoalDistance(FO_Candidates[i],vision);
                qDebug() << "Distance to Goal: "<<FinalDistance;
                /*if(GoalHeightDistance > GoalWidthDistance)
                {
                    FinalDistance = GoalWidthDistance;
                    qDebug() << i<<": WIDTH GOAL Distance: " << GoalWidthDistance;
                }
                else
                {
                    FinalDistance = GoalHeightDistance;
                    qDebug() << i<<": Height GOAL Distance: " << GoalHeightDistance;
                }*/
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
                                           bool* usedAbovehorizonCandidate,
                                           std::vector < TransitionSegment > horizontalSegments)
{
    Vector2<int> TopLeft = PossibleGoal->getTopLeft();
    Vector2<int> BottomRight = PossibleGoal->getBottomRight();
    int Colour = PossibleGoal->getColour();
    int min = TopLeft.x;
    int max = BottomRight.x;
    if((int)horizontalSegments.size() ==0 && (int)FO_AboveHorizonCandidates.size() ==0) return;

    for (int i = 0; i < (int)FO_AboveHorizonCandidates.size(); i++)
    {
        if( FO_AboveHorizonCandidates[i].getTopLeft().x > TopLeft.x - 16 &&
            FO_AboveHorizonCandidates[i].getBottomRight().x < BottomRight.x + 16 &&
            Colour == FO_AboveHorizonCandidates[i].getColour())
        {
            if(FO_AboveHorizonCandidates[i].getTopLeft().x < TopLeft.x)
            {
                TopLeft.x = FO_AboveHorizonCandidates[i].getTopLeft().x;
            }
            if(FO_AboveHorizonCandidates[i].getTopLeft().y > TopLeft.y)
            {
                TopLeft.y = FO_AboveHorizonCandidates[i].getTopLeft().y;
            }
            if(FO_AboveHorizonCandidates[i].getBottomRight().x < BottomRight.x)
            {
                BottomRight.x = FO_AboveHorizonCandidates[i].getBottomRight().x;
            }
            if(FO_AboveHorizonCandidates[i].getBottomRight().y > BottomRight.y)
            {
                BottomRight.y = FO_AboveHorizonCandidates[i].getBottomRight().y;
            }
            PossibleGoal->setTopLeft(TopLeft);
            PossibleGoal->setBottomRight(BottomRight);
            usedAbovehorizonCandidate[i] = true;
            qDebug() <<"Found OverLapping Candidate Above horizon";
        }

    }
    for (int i = (int)horizontalSegments.size(); i >= 0; i--)
    {
        //qDebug() << "Crash Check: Access HZsegs: " << i;
        TransitionSegment tempSegment = horizontalSegments[i];
        if(Colour == ClassIndex::yellow || Colour == ClassIndex::yellow_orange)
        {
            if(tempSegment.getColour() == ClassIndex::yellow || tempSegment.getColour() == ClassIndex::yellow_orange)
            {
                if(tempSegment.getStartPoint().x > min-5 && tempSegment.getEndPoint().x < max+5)
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
        else if(Colour == ClassIndex::blue || Colour == ClassIndex::shadow_blue)
        {
            if(tempSegment.getColour() == ClassIndex::blue || tempSegment.getColour() == ClassIndex::shadow_blue)
            {
                if(tempSegment.getStartPoint().x > min-5 && tempSegment.getEndPoint().x < max+5)
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
        qDebug() << i<<": " <<tempSegments[i].getStartPoint().x << "," << tempSegments[i].getStartPoint().y
                            << tempSegments[i].getEndPoint().x  << "," << tempSegments[i].getEndPoint().y  ;
        int j = i+1;
        while(tempStart.y == tempSegments[j].getStartPoint().y)
        {
            tempEnd = tempSegments[j].getEndPoint();
            j++;
        }
        i = j-1;
        if(tempEnd.x-tempStart.x > 2)
        {
            Vector2<int> tempMidPoint;
            tempMidPoint.x = (int)((tempEnd.x -tempStart.x)/2)+tempStart.x;
            tempMidPoint.y = (int)((tempEnd.y - tempStart.y)/2)+tempStart.y;
            midpoints.push_back(tempMidPoint);
            leftPoints.push_back(tempStart);
            rightPoints.push_back(tempEnd);
        }

    }
    qDebug() << "Number Of MidPoints: " <<(int) midpoints.size();
    if(midpoints.size() < 3 )
    {
        float FinalDistance;
        float GoalHeightDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (PossibleGoal.getBottomRight().y - PossibleGoal.getTopLeft().y); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
        float GoalWidthDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ ((PossibleGoal.getBottomRight().x - PossibleGoal.getTopLeft().x)*8); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS

        if(GoalHeightDistance > GoalWidthDistance)
        {
            FinalDistance = GoalWidthDistance;
            qDebug() <<": WIDTH GOAL Distance: " << GoalWidthDistance;
        }
        else
        {
            FinalDistance = GoalHeightDistance;
            qDebug() <<": Height GOAL Distance: " << GoalHeightDistance;
        }
        return FinalDistance;
    }

    //FORM EQUATION if MidPointLine
    //qDebug() << "Number Of MidPoints: " <<(int) midpoints.size();
    LSFittedLine midPointLine;
    for (int i = 0; i < (int) midpoints.size(); i++)
    {
        LinePoint  point;
        point.x = midpoints[i].x;
        point.y = midpoints[i].y;
        midPointLine.addPoint(point);
    }
    qDebug() << "Equation of Line is: " << midPointLine.getA()<< "x + " <<  midPointLine.getB() << "y + " << midPointLine.getC() << " = 0";
   /* for(int i = 0 ; i < (int)leftPoints.size(); i++)
    {
        tempDistance = (leftPoints[i].x * midPointLine.getA() + leftPoints[i].y *  midPointLine.getB() + midPointLine.getC())/math.sqrt( midPointLine.getA() *  midPointLine.getA() + midPointLine.getB() *  midPointLine.getB())
    */return distance;
}
