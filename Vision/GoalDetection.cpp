#include "GoalDetection.h"
#include "ClassificationColours.h"
#include "TransitionSegment.h"
#include "ScanLine.h"
#include "ClassifiedSection.h"
//#include <QDebug>
#include "debug.h"

GoalDetection::GoalDetection()
{
    debug<< "Vision::GoalDetection : GoalDetection Class created" << endl;
}
GoalDetection::~GoalDetection()
{
}

//! Finds the ball segments and groups updates the ball in fieldObjects (Vision is used to further classify the object)
ObjectCandidate GoalDetection::FindGoal(std::vector <ObjectCandidate>& FO_Candidates,FieldObjects* AllObjects, std::vector< TransitionSegment > horizontalSegments,Vision* vision,int height,int width)
{
	ObjectCandidate result;

	//! Go through all the candidates: to find a possible ball
	for(unsigned int i = 0; i  < FO_Candidates.size(); i++)
	{
            if(!isObjectAPossibleGoal(FO_Candidates[i]))
            {
                continue;
            }
            //qDebug() << "Crash Check: Before Extend with Horizontal Segments Detection:";
            ExtendGoalAboveHorizon(&FO_Candidates[i], horizontalSegments);
            //qDebug() << "Crash Check: Before Closely classify Detection:";
            classifyGoalClosely(&FO_Candidates[i], vision, height, width);
            //qDebug() << "Crash Check: Before Ratio check Detection:";
            bool RatioOK = isCorrectCheckRatio(FO_Candidates[i], height, width);
            //qDebug() << "RatioOK: " << RatioOK;
            int boarder = 5;

            if ( RatioOK && FO_Candidates[i].getBottomRight().x <= width-boarder &&
                        FO_Candidates[i].getBottomRight().y <= height-boarder &&
                        FO_Candidates[i].getTopLeft().x >=0+boarder &&
                        FO_Candidates[i].getTopLeft().y >=0+boarder  )
            {
                //use height
                float GoalDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (FO_Candidates[i].getBottomRight().y - FO_Candidates[i].getTopLeft().y); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
                //qDebug() << "HEIGHT GOAL Distance: " << GoalDistance;
            }
            else if(RatioOK)
            {

                //use width
                //qDebug() << vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS();
                float GoalDistance = 80* vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ ((FO_Candidates[i].getBottomRight().x - FO_Candidates[i].getTopLeft().x)*8); //GOAL_HEIGHT * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
                //qDebug() << "WIDTH GOAL Distance: " << GoalDistance;
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
void GoalDetection::ExtendGoalAboveHorizon(ObjectCandidate* PossibleGoal, std::vector < TransitionSegment > horizontalSegments)
{
    Vector2<int> TopLeft = PossibleGoal->getTopLeft();
    Vector2<int> BottomRight = PossibleGoal->getBottomRight();
    int Colour = PossibleGoal->getColour();
    int min = TopLeft.x;
    int max = BottomRight.x;
    if((int)horizontalSegments.size() ==0) return;

    for (int i = (int)horizontalSegments.size(); i >= 0; i--)
    {
        //qDebug() << "Crash Check: Access HZsegs: " << i;
        TransitionSegment tempSegment = horizontalSegments[i];
        if(Colour == ClassIndex::yellow || Colour == ClassIndex::yellow_orange)
        {
            if(tempSegment.getColour() == ClassIndex::yellow || tempSegment.getColour() == ClassIndex::yellow_orange)
            {
                if(tempSegment.getStartPoint().x > min-10 && tempSegment.getEndPoint().x < max+10)
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
    TransitionSegment* tempSeg = new TransitionSegment(SegStart,SegEnd,ClassIndex::unclassified,PossibleGoal->getColour(),ClassIndex::unclassified);
    //qDebug() << "segments (start): " << tempSeg->getStartPoint().x << "," << tempSeg->getStartPoint().y;
    ScanLine* tempLine = new ScanLine();

    int spacings = 8;
    int direction = ClassifiedSection::RIGHT;
    vision->CloselyClassifyScanline(tempLine,tempSeg,spacings, direction);

    std::vector< Vector2<int> > BallPoints;
    //qDebug() << "segments found: " << tempLine->getNumberOfSegments();
    //! Debug Output for small scans:
    int min = PossibleGoal->getTopLeft().y;
    for(int i = 0; i < tempLine->getNumberOfSegments(); i++)
    {
        tempSeg = tempLine->getSegment(i);
        //qDebug() << "segments (start): " << tempSeg->getStartPoint().x << "," << tempSeg->getStartPoint().y;
        //qDebug() << "segments (end): " << tempSeg->getEndPoint().x << "," << tempSeg->getEndPoint().y;
        if(tempSeg->getStartPoint().y < min)
        {
            min = tempSeg->getStartPoint().y;
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
