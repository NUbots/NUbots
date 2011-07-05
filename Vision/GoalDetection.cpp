#include "GoalDetection.h"
#include "ClassificationColours.h"
#include "TransitionSegment.h"
#include "ScanLine.h"
#include "ClassifiedSection.h"
#include "debug.h"
#include "Tools/Math/General.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Kinematics/Kinematics.h"
//#ifdef WIN32
//    #include <QDebug>
//#end
using namespace mathGeneral;


GoalDetection::GoalDetection()
{
    MINIMUM_GOAL_WIDTH_IN_PIXELS = 0;
    MINIMUM_GOAL_HEIGHT_IN_PIXELS = 0;
    //debug<< "Vision::GoalDetection : GoalDetection Class created" << endl;
}
GoalDetection::~GoalDetection()
{
}

//! Finds the ball segments and groups updates the goal in fieldObjects (Vision is used to further classify the object)
ObjectCandidate GoalDetection::FindGoal(std::vector <ObjectCandidate>& FO_Candidates,
                                        std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
                                        FieldObjects* AllObjects,
                                        const std::vector< TransitionSegment > &horizontalSegments,
                                        Vision* vision,int height,int width)
{
        //! Set the Minimum goal width in pixels as a function of screen width
        MINIMUM_GOAL_WIDTH_IN_PIXELS = vision->getImageWidth()/60; //6 Pixels for 320 width = 8m range
        MINIMUM_GOAL_HEIGHT_IN_PIXELS = vision->getImageHeight()/6.2; //38pixels for 240 height = 8m range
        MINIMUM_GOAL_HEIGHT_IN_PIXELS_AT_SCREEN_EDGE = 10;

        ObjectCandidate result;
        vector < ObjectCandidate > ::iterator it;
        //qDebug()<< "Candidate Size[Before Extending Above Horizon]: " <<FO_Candidates.size() << "\t Above horizon: " << FO_AboveHorizonCandidates.size();
        //! Go through all the candidates: to find a possible goal
        for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); )
	{
            if(!isObjectAPossibleGoal(*it))
            {
                //debug << "Erasing FO_CANDIDATE";
                it = FO_Candidates.erase(it);
                continue;
            }
            //debug << "Crash Check: Before Extend with Horizontal Segments Detection:";
            bool wasExtended = ExtendGoalAboveHorizon(&(*it), FO_AboveHorizonCandidates,horizontalSegments);
            if(!wasExtended)
            {
                it = FO_Candidates.erase(it);
                continue;
            }
            ++it;
        }
        //! ADD REMAINING GOAL CANDIDATES above horizon:
        FO_Candidates.insert(FO_Candidates.end(), FO_AboveHorizonCandidates.begin(), FO_AboveHorizonCandidates.end());
        FO_AboveHorizonCandidates.clear();
        //qDebug()<< "Candidate Size[Before CombineOverlapping]: " <<FO_Candidates.size();
        //for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); it++)
        //{
        //    qDebug() << (*it).getSegments().size();
        //}
        //! Combine Any "OverLapping" Candidates:
        CombineOverlappingCandidates(FO_Candidates);
        //qDebug()<< "Candidate Size[Before Ratio Size Checks]: " <<FO_Candidates.size();
        //for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); it++)
        //{
        //    qDebug() << (*it).getSegments().size();
        //}
        //! Check if the ratio of the object candidate is OK
        CheckCandidateSizeRatio(FO_Candidates, height, width);
        //qDebug()<< "Candidate Size[After Ratio Size Checks]: " <<FO_Candidates.size();
        //for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); it++)
        //{
        //    qDebug() << (*it).getSegments().size();
        //}
        //! Check if the Goal is in a Robot:
        CheckCandidateIsInRobot(FO_Candidates, AllObjects);
        //qDebug()<< "Candidate Size[After IsInRobot Checks]: " <<FO_Candidates.size();
        //for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); it++)
        //{
        //    qDebug() << (*it).getSegments().size();
        //}
        CheckIsFilled(FO_Candidates, vision);
        //qDebug()<< "Candidate Size[After Fill Check]: " <<FO_Candidates.size();
        //for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); it++)
        //{
        //    qDebug() << (*it).getSegments().size()<< (*it).getTopLeft().x << "," << (*it).getTopLeft().y <<"  "<< (*it).getBottomRight().x << "," << (*it).getBottomRight().y ;
        //}
        CheckObjectIsBelowHorizon(FO_Candidates, vision);
        //qDebug()<< "Candidate Size[After Horizon Check]: " <<FO_Candidates.size();
        //for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); it++)
        //{
        //    qDebug() << (*it).getSegments().size() << (*it).getTopLeft().x << "," << (*it).getTopLeft().y <<"  "<< (*it).getBottomRight().x << "," << (*it).getBottomRight().y ;
        //}
        //! Sort In order of Largest to Smallest:
        SortObjectCandidates(FO_Candidates);

        //! Assign FieldObjects: if more then 2 the first 2 (largest 2 posts) will be assigned left and right post
        //qDebug()<< "Candidate SORT: " <<FO_Candidates.size();

        UpdateGoalObjects(FO_Candidates, AllObjects, vision);


        return result;
}

bool GoalDetection::isObjectAPossibleGoal(const ObjectCandidate &PossibleGoal)
{
    if( PossibleGoal.getColour() == 	ClassIndex::blue ||
        PossibleGoal.getColour() == 	ClassIndex::shadow_blue ||
        PossibleGoal.getColour() == 	ClassIndex::yellow ||
        PossibleGoal.getColour() == 	ClassIndex::yellow_orange)
    {

        if(PossibleGoal.getColour() == ClassIndex::shadow_blue || PossibleGoal.getColour() == ClassIndex::blue)
        {
            int blueSize = 0;
            std::vector<TransitionSegment> segments = PossibleGoal.getSegments();
            for(unsigned int i = 0; i <segments.size(); i++ )
            {
                if(segments[i].getColour() == ClassIndex::blue)
                {
                    blueSize = blueSize + segments[i].getSize();
                }
            }
            if(blueSize > 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if(PossibleGoal.getColour() == ClassIndex::yellow || PossibleGoal.getColour() == ClassIndex::yellow_orange)
        {
            int yellowSize = 0;
            std::vector<TransitionSegment> segments = PossibleGoal.getSegments();
            for(unsigned int i = 0; i <segments.size(); i++ )
            {
                if(segments[i].getColour() == ClassIndex::yellow)
                {
                    yellowSize = yellowSize + segments[i].getSize();
                }
            }
            if(yellowSize > 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

    }
    else{
        return false;
    }
    return false;
}



bool GoalDetection::ExtendGoalAboveHorizon(ObjectCandidate* PossibleGoal,
                                           std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
                                           const std::vector < TransitionSegment > &horizontalSegments)
{
    Vector2<int> TopLeft = PossibleGoal->getTopLeft();
    Vector2<int> BottomRight = PossibleGoal->getBottomRight();
    unsigned char Colour = PossibleGoal->getColour();
    //int min = TopLeft.x;
    //int max = BottomRight.x;
    int margin = 16*1.5;
    bool hasBeenExtended = false;
    if((int)FO_AboveHorizonCandidates.size() ==0) return hasBeenExtended;

    vector < ObjectCandidate > ::iterator itAboveHorizon;
    //qDebug() << "AboveHoriCands:" << endl;
    for (itAboveHorizon = FO_AboveHorizonCandidates.begin(); itAboveHorizon < FO_AboveHorizonCandidates.end(); )
    {
        //qDebug() << itAboveHorizon->getTopLeft().x << "," << itAboveHorizon->getTopLeft().y << "\t " << itAboveHorizon->getBottomRight().x << "," << itAboveHorizon->getBottomRight().y;
        //qDebug() << TopLeft.x << BottomRight.x;
        //qDebug() << Colour << itAboveHorizon->getColour();
        if( itAboveHorizon->getTopLeft().x < TopLeft.x + margin &&
            itAboveHorizon->getBottomRight().x > BottomRight.x - margin &&
            Colour == itAboveHorizon->getColour())
        {
            if(itAboveHorizon->getTopLeft().x <= TopLeft.x)
            {
                TopLeft.x = itAboveHorizon->getTopLeft().x;
            }
            if(itAboveHorizon->getTopLeft().y <= TopLeft.y)
            {
                TopLeft.y = itAboveHorizon->getTopLeft().y;
            }
            if(itAboveHorizon->getBottomRight().x >= BottomRight.x)
            {
                BottomRight.x = itAboveHorizon->getBottomRight().x;
            }
            if(itAboveHorizon->getBottomRight().y >= BottomRight.y)
            {
                BottomRight.y = itAboveHorizon->getBottomRight().y;
            }
            PossibleGoal->setTopLeft(TopLeft);
            PossibleGoal->setBottomRight(BottomRight);
            //ADD Above Horizon segments to Possible GOAL:

            PossibleGoal->addSegments(itAboveHorizon->getSegments());
            itAboveHorizon = FO_AboveHorizonCandidates.erase(itAboveHorizon);

            hasBeenExtended = true;

            //qDebug() << "OverLapping: Join Cand " << endl;
            //usedAbovehorizonCandidate[i] = true;
            //debug <<"Found OverLapping Candidate Above horizon" << FO_AboveHorizonCandidates.size();
        }
        else
        {
            ++itAboveHorizon;
        }

    }
/*    //SCANS UP THE IMAGE
    std::vector<TransitionSegment>::reverse_iterator revIt = horizontalSegments.rbegin();
    for (; revIt != horizontalSegments.rend(); ++revIt)
    {
        //debug << "Crash Check: Access HZsegs: " << i;
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
                //debug << "Found Segment at " << tempSegment.getStartPoint().x << "," << tempSegment.getStartPoint().y;
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
                PossibleGoal->addSegment(tempSegment);
            }

        }

    }
    */
    return hasBeenExtended;
}


void GoalDetection::classifyGoalClosely(ObjectCandidate* PossibleGoal,Vision* vision)
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
    //qDebug() << "segments (start): " << tempSeg.getStartPoint().x << "," << tempSeg.getStartPoint().y ;
    ScanLine tempLine;

    int spacings = vision->getScanSpacings()/2; //8
    int direction = ScanLine::RIGHT;
    std::vector<unsigned char> colourlist;
    colourlist.reserve(3);
    if(PossibleGoal->getColour() == ClassIndex::yellow ||PossibleGoal->getColour() == ClassIndex::yellow_orange )
    {
        colourlist.push_back(ClassIndex::yellow);
        colourlist.push_back(ClassIndex::yellow_orange);
    }
    else if(PossibleGoal->getColour() == ClassIndex::blue ||PossibleGoal->getColour() == ClassIndex::shadow_blue)
    {
        colourlist.push_back(ClassIndex::blue);
        colourlist.push_back(ClassIndex::shadow_blue);
    }
    int bufferSize = 10;
    vision->CloselyClassifyScanline(&tempLine,&tempSeg,spacings, direction, colourlist,bufferSize);

    //qDebug() << "segments found: " << tempLine.getNumberOfSegments() ;
    //! Debug Output for small scans:
    int min = PossibleGoal->getTopLeft().y;
    for(int i = 0; i < tempLine.getNumberOfSegments(); i++)
    {
        TransitionSegment* tempSegment = tempLine.getSegment(i);
        //qDebug() << "segments (start): " << tempSeg.getStartPoint().x << "," << tempSeg.getStartPoint().y;
        //qDebug() << "segments (end): " << tempSeg.getEndPoint().x << "," << tempSeg.getEndPoint().y;
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
    for(it = FO_Candidates.begin(); it  < FO_Candidates.end();  )
    {
        vector < ObjectCandidate > ::iterator itInside;
        for(itInside = it+1; itInside  < FO_Candidates.end(); )
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
                it->addSegments(itInside->getSegments());
                FO_Candidates.erase(itInside);
                itInside = it+1;
                //qDebug() << "Found: Overlapping TopLeft Goal " << tempTopLeft.x << "," << tempTopLeft.y << "\t" << tempBottomRight.x <<","<< tempBottomRight.y ;
                continue;
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
                it->addSegments(itInside->getSegments());
                FO_Candidates.erase(itInside);
                itInside = it + 1;
                //qDebug() << "Found: Overlapping BottomRight Goal "   << tempTopLeft.x << "," << tempTopLeft.y << "\t" << tempBottomRight.x <<","<< tempBottomRight.y ;
                continue;
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
                it->addSegments(itInside->getSegments());
                FO_Candidates.erase(itInside);
                itInside = it + 1;

                //qDebug() << "Found: Overlapping TopLeft Goal OutSide "  << tempTopLeft.x << "," << tempTopLeft.y << "\t" << tempBottomRight.x <<","<< tempBottomRight.y ;
                continue;
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
                it->addSegments(itInside->getSegments());
                FO_Candidates.erase(itInside);
                itInside = it+1;
                //qDebug() << "Found: Overlapping BottomRight Goal OutSide "  << tempTopLeft.x << "," << tempTopLeft.y << "\t" << tempBottomRight.x <<","<< tempBottomRight.y ;
                continue;
            }
            ++itInside;
        }
        ++it;
    }
}


void GoalDetection::CheckCandidateSizeRatio(std::vector< ObjectCandidate >& FO_Candidates,int height, int width)
{
    vector < ObjectCandidate > ::iterator it;
       //! Go through all the candidates: to find a possible goal
    for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); )
    {
        //qDebug() << "Candidate: Ratio: TopLeft(x,y), BottomRight(x,y): "<< it->aspect()<< "\t" << it->getTopLeft().x << "," <<it->getTopLeft().y << "  "<<it->getBottomRight().x << ","<< it->getBottomRight().y;
        int boarder = 20; //! Boarder of pixels
        if (it->getBottomRight().x < width-boarder && it->getTopLeft().x > 0+boarder )
        {
            if(fabs(it->getBottomRight().x- it->getTopLeft().x) < MINIMUM_GOAL_WIDTH_IN_PIXELS)
            {
                //qDebug() << "Removed due to width been too small";
                it = FO_Candidates.erase(it);
                continue;
            }
        }
        if(it->getBottomRight().y < height-boarder && it->getTopLeft().y > 0+boarder )
        {
            if(fabs(it->getTopLeft().y - it->getBottomRight().y) < MINIMUM_GOAL_HEIGHT_IN_PIXELS*0.75)
            {
                //qDebug() << "Removed due to height been too small";
                it = FO_Candidates.erase(it);
                continue;
            }
        }
        if(it->getBottomRight().y < 0+boarder)
        {
            //USED TO STOP SMALL GOALS FORMING AT EDGE OF SCREEN
            if(fabs(it->getTopLeft().y - it->getBottomRight().y) < MINIMUM_GOAL_HEIGHT_IN_PIXELS_AT_SCREEN_EDGE)
            {
                //qDebug() << "Removed due to height been too small";
                it = FO_Candidates.erase(it);
                continue;
            }
            //Should not do Ratio Check if large enough, but missing top part of goal
            ++it;
            continue;
        }

        if( !isCorrectCheckRatio(*it,height, width))
        {
            it = FO_Candidates.erase(it);
            continue;
        }

        ++it;
    }
    return;
}

void GoalDetection::CheckIsFilled(std::vector< ObjectCandidate >& FO_Candidates, Vision* vision)
{
    //Calculate the minimum size of horizontal scanlines
    //Add lengths of segments of these scanlines (be a multiple of width)
    //Add lengths of transition segments in object
    //Compare and throw out "small percentages"
    vector < ObjectCandidate > ::iterator it;
    for(it =  FO_Candidates.begin(); it  < FO_Candidates.end(); )
    {

        int heigtOfPossibleGoal = it->height();
        int widthOfPossibleGoal = it->width();

        int horizontalScanspacing = vision->getScanSpacings();
        float minIntersectingScanlines = heigtOfPossibleGoal / (float)horizontalScanspacing; //Above horizon ScanSpacing, not inbetween horizon.

        int maxScanLengthOfMinScanlines = minIntersectingScanlines * widthOfPossibleGoal;

        vector<TransitionSegment> segments = it->getSegments();
        int lengthsOfSegments = 0;
        int numberOfHorizontalScanLines = 0;
        for(unsigned int i = 0; i < segments.size(); i++)
        {
            lengthsOfSegments = lengthsOfSegments + segments[i].getSize();
            //qDebug() << segments[i].getSize() << segments[i].getStartPoint().x <<", "<<segments[i].getStartPoint().y  << segments[i].getEndPoint().x<< ", "<< segments[i].getEndPoint().y<< (MINIMUM_GOAL_WIDTH_IN_PIXELS/2 +2);

            if(segments[i].getSize() > (MINIMUM_GOAL_WIDTH_IN_PIXELS/2 +2) && segments[i].getStartPoint().y == segments[i].getEndPoint().y)
            {
                numberOfHorizontalScanLines++;
            }
        }

        //qDebug() << "Comparing LengthOfMinScanIntersection and actual lengths: " <<maxScanLengthOfMinScanlines << lengthsOfSegments << heigtOfPossibleGoal << widthOfPossibleGoal << segments.size() << numberOfHorizontalScanLines;
        if(lengthsOfSegments <  maxScanLengthOfMinScanlines*0.5 || numberOfHorizontalScanLines <= 1)
        {
            it = FO_Candidates.erase(it);
            continue;
        }
        else
        {
            ++it;
        }
    }
}

bool GoalDetection::isCorrectCheckRatio(ObjectCandidate PossibleGoal,int height, int width)
{
    //debug << "Checking Ratio: " << PossibleBall.aspect();

    //! Check if at Edge of Screen, if so continue with other checks, otherwise, look at ratio and check if in thresshold
    int boarder = 16; //! Boarder of pixels
    if (PossibleGoal.getBottomRight().x < width-boarder &&
        PossibleGoal.getBottomRight().y < height-boarder &&
        PossibleGoal.getTopLeft().x > 0+boarder &&
        PossibleGoal.getTopLeft().y > 0+boarder  )
    {
        //POSSIBLE GOALS ARE:
        //      Objects which have grouped segments,
        //      or objects with one segment, but very small (still like to consider).
        if((PossibleGoal.aspect() > 0 && PossibleGoal.aspect() < 1 )|| PossibleGoal.aspect()==0)
        {

            return true;
        }
        else
        {
            //debug << "Thrown out due to incorrect ratio";
            return false;
        }
    }
    else
    {
        //debug << "Returned True at edge of screen";
        return true;
    }
}

void  GoalDetection::CheckCandidateIsInRobot(std::vector<ObjectCandidate>& FO_Candidates, FieldObjects* AllObjects)
{
    vector < ObjectCandidate > ::iterator it;
    bool objectRemoved;

       //! Go through all the candidates: to find a possible goal
    for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); )
    {
        objectRemoved = false;

        if(it->getColour() != ClassIndex::blue && it->getColour() != ClassIndex::shadow_blue )
        {
            ++it;
            continue;
        }
        //qDebug() << "Checking  Goal: BLUE" ;
        Vector2<int> topLeft = it->getTopLeft();
        Vector2<int> bottomRight = it->getBottomRight();
        vector < AmbiguousObject > ::iterator FO_it;
        for (FO_it = AllObjects->ambiguousFieldObjects.begin();  FO_it  < AllObjects->ambiguousFieldObjects.end(); FO_it++)
        {
            //qDebug() << "Checking  Robot: " << FO_it->getID();
            if(FO_it->getID() != FieldObjects::FO_BLUE_ROBOT_UNKNOWN) continue;
            Vector2<int> robotTopLeft, robotBottomRight;
            int widthbuffer = FO_it->getObjectWidth()*0.1;
            robotTopLeft.x = FO_it->ScreenX() -  FO_it->getObjectWidth()/2 - widthbuffer;
            robotTopLeft.y = FO_it->ScreenY() -  FO_it->getObjectHeight()/2 - FO_it->getObjectHeight();
            robotBottomRight.x = FO_it->ScreenX() +  FO_it->getObjectWidth()/2 + widthbuffer;
            robotBottomRight.y = FO_it->ScreenY() +  FO_it->getObjectHeight()/2;
            //qDebug() << "Checking Goal inside Robot: " << topLeft.x << robotTopLeft.x << topLeft.y << robotTopLeft.y << bottomRight.x << robotBottomRight.x <<  bottomRight.y << robotBottomRight.y;
            if( topLeft.x >= robotTopLeft.x && topLeft.y >= robotTopLeft.y && bottomRight.x <= robotBottomRight.x &&  bottomRight.y <= robotBottomRight.y)
            {
                //qDebug() << "Errasing Goal because inside Robot";
                objectRemoved = true;
                it = FO_Candidates.erase(it);
                break;
            }
        }
        if(objectRemoved)
        {
            continue;
        }
        else
        {
            ++it;
        }
    }
    return;
}

void GoalDetection::CheckObjectIsBelowHorizon(std::vector<ObjectCandidate>& FO_Candidates, Vision* vision)
{
    vector < ObjectCandidate > ::iterator it;
    for(it = FO_Candidates.begin(); it  < FO_Candidates.end(); )
    {
        int buffer = 20;
        if((vision->m_horizonLine.IsBelowHorizon(it->getBottomRight().x, it->getBottomRight().y + buffer))== false)
        {
            //qDebug() << "Removing Goal Above Horizon:" << it->getBottomRight().x<< ","<< it->getBottomRight().y << vision->m_horizonLine.findYFromX(it->getBottomRight().x);
            //qDebug() << "Horizon Information: " << vision->m_horizonLine.getGradient() << "x + " << vision->m_horizonLine.getYIntercept();
            //qDebug() << vision->m_horizonLine.getA() << "x + "<< vision->m_horizonLine.getB() << "y + " << vision->m_horizonLine.getC();
            it = FO_Candidates.erase(it);
            continue;
        }

        ++it;

    }
    return;
}


Vector3<float> GoalDetection::FindGoalSphericalPosition( const ObjectCandidate &PossibleGoal, Vision* vision)
{
    float distance = 0.0;
    std::vector < TransitionSegment > tempSegments = PossibleGoal.getSegments();
    std::vector < Vector2<int> > midpoints, leftPoints, rightPoints;
    Vector2<int> tempStart, tempEnd;
    float pixelError = 0.0;


    //! USE CADIDATE WIDTH:

    //! Calculate the soft colour list:
    std::vector <unsigned char> colourlist;
    colourlist.reserve(2);
    if(PossibleGoal.getColour() == ClassIndex::blue || PossibleGoal.getColour() == ClassIndex::shadow_blue)
    {
        //qDebug() << "BlueGoal Found: addding blue softColours to list" ;
        colourlist.push_back(ClassIndex::blue);
        colourlist.push_back(ClassIndex::shadow_blue);
    }
    else if (PossibleGoal.getColour() == ClassIndex::yellow || PossibleGoal.getColour() == ClassIndex::yellow_orange)
    {
        colourlist.push_back(ClassIndex::yellow);
        colourlist.push_back(ClassIndex::yellow_orange);
    }
    // Joins segments on same scanline and finds MIDPOINTS, leftPoints and rightPoints: Finding the last segment in the same scanline points in the same scan line
    for (int i =0; i< (int)tempSegments.size(); i++)
    {
        tempStart = tempSegments[i].getStartPoint();
        tempEnd = tempSegments[i].getEndPoint();

        if(tempStart.x == tempEnd.x && tempStart.y != tempEnd.y) continue; //! Throw out vertical lines

        int j = i+1;
        if(j < (int)tempSegments.size())
        {
            while(tempStart.y == tempSegments[j].getStartPoint().y)
            {
                //Scan is backwards, so if a segment is found, we should up date the start point, not the end point
                tempStart = tempSegments[j].getStartPoint();
                j++;
                if(j >= (int)tempSegments.size())
                {
                    break;
                }
            }
        }

        i = j-1;
        //! Removes Small and "Top segments = cross bar"

        if(fabs(tempEnd.x-tempStart.x) > 2 && i < (int)tempSegments.size())
        {
            Vector2<int> tempMidPoint;
            //FIND the EXACT TEMPEND and TEMPSTART points:
            int checkEndx = tempEnd.x;
            int checkStartx = tempStart.x;

            //Checking Start of Transition: Go Backwards if current colour is valid, otherwise go forwards
            if(vision->isValidColour(vision->classifyPixel(tempStart.x,tempStart.y),colourlist))
            {
                //Find the pixel which isnt the colour
                while(vision->isPixelOnScreen(checkStartx,tempStart.y) && vision->isValidColour(vision->classifyPixel(checkStartx,tempStart.y),colourlist) )
                {
                    checkStartx--;
                }
            }
            else
            {
                while(vision->isPixelOnScreen(checkStartx,tempStart.y) && !vision->isValidColour(vision->classifyPixel(checkStartx,tempStart.y),colourlist))
                {
                    checkStartx++;
                }
            }
            tempStart.x = checkStartx;
            //Checking End of Transition: Go forwards if current colour is valid, otherwise go backwards
            //qDebug() << "Colour At End: "<< vision->classifyPixel(tempEnd.x,tempEnd.y);
            if(vision->isValidColour(vision->classifyPixel(tempEnd.x,tempEnd.y),colourlist))
            {
                //Find the pixel which isnt the colour
                while(vision->isPixelOnScreen(checkEndx,tempEnd.y) && vision->isValidColour(vision->classifyPixel(checkEndx,tempEnd.y),colourlist))
                {
                    checkEndx++;
                }
            }
            else
            {
                while(vision->isPixelOnScreen(checkEndx,tempEnd.y) && !vision->isValidColour(vision->classifyPixel(checkEndx,tempEnd.y),colourlist))
                {
                    checkEndx--;
                }
            }
            tempEnd.x = checkEndx;
            //qDebug() << "Start, End: "<< i <<":" << tempStart.x << ", " << tempStart.y << "\t" <<  tempEnd.x << ", " << tempEnd.y << "\t" << tempEnd.x - tempStart.x << MINIMUM_GOAL_WIDTH_IN_PIXELS/2;
            if(fabs(tempEnd.x -tempStart.x) < MINIMUM_GOAL_WIDTH_IN_PIXELS/2) continue;


            tempMidPoint.x = (int)((tempEnd.x +tempStart.x)/2);
            tempMidPoint.y = (int)((tempEnd.y +tempStart.y)/2);
            midpoints.push_back(tempMidPoint);
            leftPoints.push_back(tempStart);
            rightPoints.push_back(tempEnd);
        }

    }
    //qDebug() << "Number Of MidPoints: " <<(int) midpoints.size();

    //Check if the top is cut off:
    //qDebug() << "Condition Check: "<<PossibleGoal.getTopLeft().y << vision->getScanSpacings()/2 << (PossibleGoal.getTopLeft().y < 0 +  vision->getScanSpacings()/2);
    int MIN_MIDPOINTS, LAST_MIDPOINT;

    //IT IS CUT OFF: (NO CROSSBAR) Removed by method further down
    MIN_MIDPOINTS = 2;
    LAST_MIDPOINT = midpoints.size();

    LSFittedLine midPointLine;
    LSFittedLine leftPointLine;
    LSFittedLine rightPointLine;

    //FIND AVERAGE WIDTH:
    float averageWidth = 0;
    float sum = 0;
    for(int i =0; i < LAST_MIDPOINT; i++)
    {
        sum = sum + fabs(rightPoints[i].x - leftPoints[i].x);
    }
    averageWidth = (float)sum/(float)rightPoints.size();
    for(int i =0; i < LAST_MIDPOINT; i++)
    {
        float width = fabs(rightPoints[i].x - leftPoints[i].x);
        if(averageWidth + 5 > width)
        {
            LinePoint point;
            point.x = midpoints[i].x;
            point.y = midpoints[i].y;
            midPointLine.addPoint(point);
            point.x = leftPoints[i].x;
            point.y = leftPoints[i].y;
            leftPointLine.addPoint(point);
            point.x = rightPoints[i].x;
            point.y = rightPoints[i].y;
            rightPointLine.addPoint(point);
        }
    }

/*
    //Form a line with midpoints
    for (int i = 0; i < (int) LAST_MIDPOINT; i++)
    {

        LinePoint  point;
        point.x = midpoints[i].x;
        point.y = midpoints[i].y;
        midPointLine.addPoint(point);
        qDebug() << midPointLine.getMSD();
    }

    //check distance between midpoint line with left and right points
    for (int i = 0; i < (int) LAST_MIDPOINT; i++)
    {
        float d1 = DistanceLineToPoint(midPointLine,leftPoints[i]);
        float d2 = DistanceLineToPoint(midPointLine,rightPoints[i]);
        qDebug() << d1 << d2 << fabs(d1-d2) << d1+d2;
        if(fabs(d1-d2) < 5)
        {
            LinePoint  point;
            point.x = leftPoints[i].x;
            point.y = leftPoints[i].y;
            leftPointLine.addPoint(point);
            point.x = rightPoints[i].x;
            point.y = rightPoints[i].y;
            rightPointLine.addPoint(point);
        }

    }
*/

    //qDebug() << "Equation of MidLine is: " << midPointLine.getA()<< "x + " <<  midPointLine.getB() << "y  = " << midPointLine.getC() << endl;
    //qDebug()<< "Interescting Screen at TOP: \t"<< midPointLine.findXFromY(0)<< ","<< 0 << endl;
    //qDebug()<< "Interescting Screen at Bottom: \t"<< midPointLine.findXFromY(240)<< ","<< 240 << endl;

    //qDebug() << "Equation of LeftLine is: " << leftPointLine.getA()<< "x + " <<  leftPointLine.getB() << "y = " << leftPointLine.getC()  << endl;
    //qDebug()<< "Interescting Screen at TOP: \t"<< leftPointLine.findXFromY(0)<< ","<< 0 << endl;
    //qDebug()<< "Interescting Screen at Bottom: \t"<< leftPointLine.findXFromY(240)<< ","<< 240 << endl;

    //qDebug() << "Equation of RightLine is: " << rightPointLine.getA()<< "x + " <<  rightPointLine.getB() << "y = " << rightPointLine.getC() << endl;
    //qDebug()<< "Interescting Screen at TOP: \t"<< rightPointLine.findXFromY(0)<< ","<< 0 << endl;
    //qDebug()<< "Interescting Screen at Bottom: \t"<< rightPointLine.findXFromY(240)<< ","<< 240 << endl;

    Vector2<int> leftpoint;
    leftpoint.y = (PossibleGoal.getBottomRight().y + PossibleGoal.getTopLeft().y)/2;
    leftpoint.x = leftPointLine.findXFromY(leftpoint.y);

    float DistanceCentre = DistanceLineToPoint(rightPointLine,leftpoint);
    //leftpoint.x = leftPointLine.findXFromY(240);
    //leftpoint.y = 240;
    //float DistanceBottom = DistanceLineToPoint(rightPointLine,leftpoint);

    //qDebug() << "Distance at centre: " << DistanceCentre;

    float widthSum = 0;
    float tightwidthSum = 0;
    int tightPoints = 0;
    //float largestWidth = 0;
    float tightlargestWidth = 0;
    float largestWidthRight = 0;
    float largestWidthLeft = 0;

    //! Average filter: All Mid-Points
    //! Tight Average filter: Using the principle that mid-points should have symetrical left and right distances,
    //! we can filter mid-points which have bad left and right distances, by looking at the difference between left and right, as they should be "approx 0".
    //! Largest Width is obtained by itterating through the midpoint distances, and obtaining the largest width that has symetrical left and right distances.
    for(int i = 0 ; i < (int)leftPoints.size()-1; i++)
    {
        Vector2<int> leftpoint = leftPoints[i];
        Vector2<int> rightpoint = rightPoints[i];
        float leftPixels = DistanceLineToPoint(midPointLine, leftpoint);
        float rightPixels = DistanceLineToPoint(midPointLine, rightpoint);

        //if(leftPixels+rightPixels < MINIMUM_GOAL_WIDTH_IN_PIXELS/2) continue;

        widthSum +=  leftPixels + rightPixels;

        //Check if the current width is larger then the largest width and pixels are close to the mid line
        if(fabs(leftPixels - rightPixels) < (leftPixels + rightPixels) * 0.5)
        {
            tightwidthSum +=  leftPixels + rightPixels;
            tightPoints++;
            if(tightlargestWidth < leftPixels + rightPixels)
            {
                tightlargestWidth = leftPixels + rightPixels;
            }
            if(rightPixels >largestWidthRight)
            {
                largestWidthRight = rightPixels;
            }
            if(leftPixels > largestWidthLeft)
            {
                largestWidthLeft = leftPixels;
            }
        }
        /*if(largestWidth < leftPixels + rightPixels)
        {
            largestWidth = leftPixels + rightPixels;
        }*/
        //qDebug() << DistanceLineToPoint(midPointLine, leftpoint) << ", "<< DistanceLineToPoint(midPointLine, rightpoint) <<  " = "<< DistanceLineToPoint(midPointLine, leftpoint) +  DistanceLineToPoint(midPointLine, rightpoint) ;

    }

    //! Width Averaging:
    //qDebug() << (float)(leftPoints.size()-1);
    widthSum = widthSum/ (float)(leftPoints.size()-1);
    if(tightPoints > 0)
    {
        tightwidthSum = tightwidthSum/tightPoints;
    }
    else
    {
        tightwidthSum = 0;
    }

    //! Pick the which Goal Width Distance to use:
    
    //qDebug() << "Comparing: Largest: "<< GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ largestWidth << "\tAverage: "<<GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ widthSum << "\tTighter Average: " << GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ tightwidthSum << "\tTighter Large fit: " << GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ tightlargestWidth ;
    //qDebug() << "Comparing with Error: Largest: "<< GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (largestWidth +pixelError)<< "\tAverage: "<<GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (widthSum +pixelError)<< "\tTighter Average: " << GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (tightwidthSum + +pixelError)<< "\tTighter Large fit: " << GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (tightlargestWidth + +pixelError);

    //if(tightlargestWidth  == 0 &&  widthSum < MINIMUM_GOAL_WIDTH_IN_PIXELS * 2 && tightwidthSum < MINIMUM_GOAL_WIDTH_IN_PIXELS * 2)

    /*if(( widthSum < MINIMUM_GOAL_WIDTH_IN_PIXELS * 2 || midpoints.size() < 5) && largestWidth < widthSum * 1.2 && largestWidth != 0 && largestWidth > widthSum )
    {
        distance = GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (largestWidth + pixelError); //GOAL_WIDTH * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
        qDebug() << "Largest MidPoints Distance:" << distance << "cm using " << largestWidth << " pixels.";
    }*/

    if(tightwidthSum != 0 && DistanceCentre !=0)
    {
        distance = GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (DistanceCentre + pixelError);
    }
    else if (tightwidthSum > 0 )
    {
        distance = GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (tightwidthSum+ pixelError); //GOAL_WIDTH * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
        //qDebug() << "Tight Average MidPoints Distance:" << distance << "cm using " << tightwidthSum << "pixels.";
    }
    else
    {
        distance = GOAL_WIDTH * vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()/ (widthSum + pixelError); //GOAL_WIDTH * EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS
        //qDebug() << "Average MidPoints Distance:" << distance << "cm using " << widthSum << "pixels.";
    }

    /*
    float D2Pdistance = DistanceToPoint(PossibleGoal,vision);

    qDebug() << "After Average Distance to Bottom Of Goals: Width:"<< distance << ", D2PDistance: " << D2Pdistance;
    float distanceBuffer = 0;
    if(distance > D2Pdistance + distanceBuffer)
    {
        distance = D2Pdistance;
    }
    */

    //To the bottom of the Goal Post.
    Vector2<int> bottomCentrePoint;
    bottomCentrePoint.y = PossibleGoal.getBottomRight().y;
    bottomCentrePoint.x = midPointLine.findXFromY(bottomCentrePoint.y);


    Vector3<float> sphericalPosition;
    float bearing = (float)vision->CalculateBearing(bottomCentrePoint.x);
    float elevation = (float)vision->CalculateElevation(bottomCentrePoint.y);

    sphericalPosition[0] = distance;//distance
    sphericalPosition[1] = bearing;
    sphericalPosition[2] = elevation;

    //qDebug() << "Goal Info: (D,B,E): "<<  distance << "\t" << bearing << "\t" << elevation;
    return sphericalPosition;
}
float GoalDetection::DistanceToPoint(const ObjectCandidate &PossibleGoal, Vision* vision)
{
    //USING DISTANCE TO POINT
    //get the center point of the of GOAL Post:
    float D2Pdistance = 100000;
    float MiddleX = (PossibleGoal.getTopLeft().x + PossibleGoal.getBottomRight().x)/2;
    float BottomY = PossibleGoal.getBottomRight().y;

    float bearing = vision->CalculateBearing(MiddleX);
    float elevation = vision->CalculateElevation(BottomY);

    //! @todo TODO: the camera to ground transform is going to be made private in a future release.
    // I think Kinematics::DistanceToPoint should be a friend with the sensor data, and get the transform itself
    // Also will need to be updated when the sensor data can properly store kinematic data
    vector<float> ctgvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector); 
    if(isOK == true)
    {
        Matrix camera2groundTransform = Matrix4x4fromVector(ctgvector);
        Vector3<float> result;
        result = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
        D2Pdistance = result[0];
        bearing = result[1];
        elevation = result[2];

        #if DEBUG_VISION_VERBOSITY > 6
            debug << "\t\tCalculated Distance to Point: " << *distance<<endl;
        #endif
    }
    return D2Pdistance;
}

float GoalDetection::DistanceLineToPoint(const LSFittedLine &midPointLine, const Vector2<int> & point)
{
    float distance = fabs( point.x * midPointLine.getA() + point.y *  midPointLine.getB() - midPointLine.getC())
                   / sqrt( midPointLine.getA() *  midPointLine.getA() + midPointLine.getB() *  midPointLine.getB());
    //debug << "Distance L2P: " << distance << "\t Using: "<<point.x << ","<< point.y<<endl;
    return distance;
}

void GoalDetection::SortObjectCandidates(std::vector<ObjectCandidate>& FO_Candidates)
{
    /*for(unsigned int i = 0; i < FO_Candidates.size(); i++)
    {
        debug << i <<":" << FO_Candidates[i].width()*FO_Candidates[i].height();
    }*/

    std::sort(FO_Candidates.begin(), FO_Candidates.end(), ObjectCandidateSizeSortPredicate);

    /*for(unsigned int i = 0; i < FO_Candidates.size(); i++)
    {
        debug << i <<":" << FO_Candidates[i].width()*FO_Candidates[i].height();
    }*/
    return;
}


bool GoalDetection::ObjectCandidateSizeSortPredicate(const ObjectCandidate& goal1, const ObjectCandidate& goal2)
{
    return goal1.width()*goal1.height() > goal2.width()*goal2.height();
}

void GoalDetection::UpdateAFieldObject(FieldObjects* AllObjects, Vision* vision, ObjectCandidate* GoalPost , int ID, Vector3<float> sphericalPosition)
{

    Vector2<int> viewPosition;
    Vector2<int> sizeOnScreen;
    Vector3<float> sphericalError;

    viewPosition.x = GoalPost->getCentreX();
    viewPosition.y = GoalPost->getCentreY();
    Vector3 <float> transformedSphericalPosition;
    Vector2<float> screenPositionAngle(sphericalPosition[1], sphericalPosition[2]);
    
    //! @todo TODO: the camera to ground transform is going to be made private in a future release.
    // I think Kinematics::DistanceToPoint should be a friend with the sensor data, and get the transform itself
    // Also will need to be updated when the sensor data can properly store kinematic data
    vector<float> ctvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraTransform, ctvector);
    if(isOK == true)
    {
        Matrix cameraTransform = Matrix4x4fromVector(ctvector);
        transformedSphericalPosition = Kinematics::TransformPosition(cameraTransform,sphericalPosition);
    }

    sizeOnScreen.x = GoalPost->width();
    sizeOnScreen.y = GoalPost->height();
    AllObjects->stationaryFieldObjects[ID].UpdateVisualObject(      transformedSphericalPosition,
                                                                    sphericalError,
                                                                    screenPositionAngle,
                                                                    viewPosition,
                                                                    sizeOnScreen,
                                                                    vision->m_timestamp);
    return;
}

/**
*
* Function Name:	PostProcessGoalPosts
* Author:		Aaron Wong
* Description: 		Post processing of Goal Posts to remove blue and yellow posts seen in an image.
*
* Date:
* input: 		NA
* Output:		NA
*/
void GoalDetection::PostProcessGoalPosts(FieldObjects* AllObjects)
{
    //Empty Variables:

    //! Find the Colour of closest goal:
    unsigned int colour = FindColourOfClosestPost(AllObjects);
    //qDebug() << "Closest Colour is:" << colour;
    //! Removing Objects which are the opposite colour:
    if (colour == ClassIndex::unclassified)
    {
        return;
    }
    else if( colour == ClassIndex::blue)
    {
        //qDebug() << "Removing ALL YELLOW POSTS";
        //! Remove All Yellow Posts:
        if( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() == true)
        {
            AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].setIsVisible(false);
        }
        if( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible()== true)
        {
            AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].setIsVisible(false);
        }
        for(unsigned int i = 0; i < AllObjects->ambiguousFieldObjects.size(); i++)
        {
            if(AllObjects->ambiguousFieldObjects[i].isObjectVisible() == true)
            {
                if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
                {
                    AllObjects->ambiguousFieldObjects[i].setIsVisible(false);
                }
            }
        }
    }
    else if(colour == ClassIndex::yellow)
    {
        //! Remove all Blue posts:
        //qDebug() << "Removing ALL BLUE POSTS";
        if( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() == true)
        {
            AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].setIsVisible(false);
        }
        if( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible() == true)
        {
            AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].setIsVisible(false);
        }
        for(unsigned int i = 0; i < AllObjects->ambiguousFieldObjects.size(); i++)
        {
            if(AllObjects->ambiguousFieldObjects[i].isObjectVisible() == true)
            {
                if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
                {
                    AllObjects->ambiguousFieldObjects[i].setIsVisible(false);
                }
            }
        }

    }
}

unsigned char GoalDetection::FindColourOfClosestPost(FieldObjects* AllObjects)
{
    unsigned char colour = ClassIndex::unclassified;
    int closestDistance = 2000;
    //! CHECK FOR ALL KNOWN POSTS
    for(unsigned int i = FieldObjects::FO_BLUE_LEFT_GOALPOST; i <=  FieldObjects::FO_YELLOW_RIGHT_GOALPOST; i++)
    {
        if(AllObjects->stationaryFieldObjects[i].isObjectVisible() == true)
        {
            if(AllObjects->stationaryFieldObjects[i].measuredDistance() < closestDistance)
            {
                closestDistance = AllObjects->stationaryFieldObjects[i].measuredDistance();
                if(AllObjects->stationaryFieldObjects[i].getID() == FieldObjects::FO_BLUE_LEFT_GOALPOST
                   || AllObjects->stationaryFieldObjects[i].getID() == FieldObjects::FO_BLUE_RIGHT_GOALPOST )
                {
                    colour = ClassIndex::blue;
                }
                else
                {
                    colour = ClassIndex::yellow;
                }
            }
        }
    }
    //! CHECK FOR ALL AMBIGUOUS POSTS
    for(unsigned int i = 0; i < AllObjects->ambiguousFieldObjects.size(); i++)
    {
        if(AllObjects->ambiguousFieldObjects[i].isObjectVisible() == true)
        {
            if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
            {
                if(AllObjects->ambiguousFieldObjects[i].measuredDistance() < closestDistance)
                {
                    closestDistance = AllObjects->ambiguousFieldObjects[i].measuredDistance();
                    colour = ClassIndex::blue;

                }
            }
            else if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
            {
                if(AllObjects->ambiguousFieldObjects[i].measuredDistance() < closestDistance)
                {
                    closestDistance = AllObjects->ambiguousFieldObjects[i].measuredDistance();
                    colour = ClassIndex::yellow;
                }
            }
        }
    }
    //qDebug() << "Closest Distance: " << closestDistance << "\t"<< colour;
    return colour;
}

Vector3<float> GoalDetection::CalculateSphericalPosition(ObjectCandidate* GoalPost, Vision* vision)
{
    Vector3<float> sphericalPosition;
    classifyGoalClosely(GoalPost, vision);

    //float bearing = (float)vision->CalculateBearing(GoalPost->getCentreX());
    //float elevation = (float)vision->CalculateElevation(GoalPost->getCentreY());
    //qDebug()<< "OLD B,E: " << bearing << "\t" <<elevation;
    sphericalPosition = FindGoalSphericalPosition((*GoalPost),vision);
    //sphericalPosition[1] = bearing;
    //sphericalPosition[2] = elevation;
    return sphericalPosition;
}

void GoalDetection::AddAmbiguousGoalPost(ObjectCandidate* GoalPost, FieldObjects* AllObjects, Vision* vision)
{
    AmbiguousObject newAmbObj = AmbiguousObject();
    //Assign Possible IDs: Yellow or Blue, Left or Right Posts
    if(GoalPost->getColour() == ClassIndex::blue || GoalPost->getColour() == ClassIndex::shadow_blue)
    {
        newAmbObj = AmbiguousObject(FieldObjects::FO_BLUE_GOALPOST_UNKNOWN, "Unknown Blue Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);

    }
    else if(GoalPost->getColour() == ClassIndex::yellow || GoalPost->getColour()  == ClassIndex::yellow_orange)
    {
        newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);

    }
    else{
        return;
    }

    Vector2<int> viewPosition;
    Vector2<int> sizeOnScreen;
    Vector3<float> sphericalError;
    Vector3<float> sphericalPosition = CalculateSphericalPosition(GoalPost, vision);
    viewPosition.x = GoalPost->getCentreX();
    viewPosition.y = GoalPost->getCentreY();
    Vector3 <float> transformedSphericalPosition;
    Vector2<float> screenPositionAngle(sphericalPosition[1], sphericalPosition[2]);
    
    vector<float> ctvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraTransform, ctvector);
    if(isOK == true)
    {
        Matrix cameraTransform = Matrix4x4fromVector(ctvector);
        transformedSphericalPosition = Kinematics::TransformPosition(cameraTransform,sphericalPosition);
    }

    sizeOnScreen.x = GoalPost->width();
    sizeOnScreen.y = GoalPost->height();

    newAmbObj.UpdateVisualObject(   transformedSphericalPosition,
                                    sphericalError,
                                    screenPositionAngle,
                                    viewPosition,
                                    sizeOnScreen,
                                    vision->m_timestamp);

    AllObjects->ambiguousFieldObjects.push_back(newAmbObj);
}

void GoalDetection::UpdateGoalObjects(vector < ObjectCandidate > FO_Candidates, FieldObjects* AllObjects, Vision* vision)
{

    if(FO_Candidates.size() >= 2 && FO_Candidates[0].getCentreX() < FO_Candidates[1].getCentreX() )
    {
        if((FO_Candidates[0].getColour() == ClassIndex::blue || FO_Candidates[0].getColour() == ClassIndex::shadow_blue) &&
           (FO_Candidates[1].getColour() == ClassIndex::blue || FO_Candidates[1].getColour() == ClassIndex::shadow_blue) )
        {
            //qDebug()<< "Updating FO: Posts A";
            Vector3<float> SphericalPosition0 = CalculateSphericalPosition(&FO_Candidates[0],vision);
            Vector3<float> SphericalPosition1 = CalculateSphericalPosition(&FO_Candidates[1],vision);
            if(fabs(SphericalPosition1[0] - SphericalPosition0[0]) < 1.5 * DISTANCE_BETWEEN_POSTS)
            {
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[0], FieldObjects::FO_BLUE_LEFT_GOALPOST,SphericalPosition0 );
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[1], FieldObjects::FO_BLUE_RIGHT_GOALPOST,SphericalPosition1);
            }
            else
            {
                AddAmbiguousGoalPost(&FO_Candidates[0], AllObjects, vision);
                AddAmbiguousGoalPost(&FO_Candidates[1], AllObjects, vision);
            }

        }
        else if ((FO_Candidates[0].getColour() == ClassIndex::yellow || FO_Candidates[0].getColour() == ClassIndex::yellow_orange) &&
                 (FO_Candidates[1].getColour() == ClassIndex::yellow || FO_Candidates[1].getColour() == ClassIndex::yellow_orange) )
        {
            //qDebug()<< "Updating FO: Posts B";
            Vector3<float> SphericalPosition0 = CalculateSphericalPosition(&FO_Candidates[0],vision);
            Vector3<float> SphericalPosition1 = CalculateSphericalPosition(&FO_Candidates[1],vision);
            if(fabs(SphericalPosition1[0] - SphericalPosition0[0]) < 1.5 * DISTANCE_BETWEEN_POSTS)
            {
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[0], FieldObjects::FO_YELLOW_LEFT_GOALPOST,SphericalPosition0 );
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[1], FieldObjects::FO_YELLOW_RIGHT_GOALPOST,SphericalPosition1 );
            }
            else
            {
                AddAmbiguousGoalPost(&FO_Candidates[0], AllObjects, vision);
                AddAmbiguousGoalPost(&FO_Candidates[1], AllObjects, vision);
            }
        }
    }
    else if (FO_Candidates.size() >= 2 && FO_Candidates[0].getCentreX() > FO_Candidates[1].getCentreX())
    {
        if((FO_Candidates[0].getColour() == ClassIndex::blue || FO_Candidates[0].getColour() == ClassIndex::shadow_blue) &&
           (FO_Candidates[1].getColour() == ClassIndex::blue || FO_Candidates[1].getColour() == ClassIndex::shadow_blue) )
        {
            //qDebug()<< "Updating FO: Posts C";
            Vector3<float> SphericalPosition0 = CalculateSphericalPosition(&FO_Candidates[0],vision);
            Vector3<float> SphericalPosition1 = CalculateSphericalPosition(&FO_Candidates[1],vision);
            if(fabs(SphericalPosition1[0] - SphericalPosition0[0]) < 1.5 * DISTANCE_BETWEEN_POSTS)
            {
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[0], FieldObjects::FO_BLUE_RIGHT_GOALPOST,SphericalPosition0 );
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[1], FieldObjects::FO_BLUE_LEFT_GOALPOST,SphericalPosition1 );
            }
            else
            {
                AddAmbiguousGoalPost(&FO_Candidates[0], AllObjects, vision);
                AddAmbiguousGoalPost(&FO_Candidates[1], AllObjects, vision);
            }
        }
        else if ((FO_Candidates[0].getColour() == ClassIndex::yellow || FO_Candidates[0].getColour() == ClassIndex::yellow_orange) &&
                 (FO_Candidates[1].getColour() == ClassIndex::yellow || FO_Candidates[1].getColour() == ClassIndex::yellow_orange) )
        {
            //qDebug()<< "Updating FO: Posts D";
            Vector3<float> SphericalPosition0 = CalculateSphericalPosition(&FO_Candidates[0],vision);
            Vector3<float> SphericalPosition1 = CalculateSphericalPosition(&FO_Candidates[1],vision);
            if(fabs(SphericalPosition1[0] - SphericalPosition0[0]) < 1.5 * DISTANCE_BETWEEN_POSTS)
            {
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[0], FieldObjects::FO_YELLOW_RIGHT_GOALPOST,SphericalPosition0 );
                UpdateAFieldObject(AllObjects,vision,&FO_Candidates[1], FieldObjects::FO_YELLOW_LEFT_GOALPOST,SphericalPosition1 );
            }
            else
            {
                AddAmbiguousGoalPost(&FO_Candidates[0], AllObjects, vision);
                AddAmbiguousGoalPost(&FO_Candidates[1], AllObjects, vision);
            }
        }
    }
    //qDebug()<< "Finisihed Updating FO: Posts";
    vector < ObjectCandidate > ::iterator it;
    for (it = FO_Candidates.begin(); it  < FO_Candidates.end(); )
    {
        //! SKIP first 2 objects if greater then size is greater or equal then 2!
        if(FO_Candidates.size() > 2 && it == FO_Candidates.begin())
        {
            ++it;
            ++it;
        }
        if(FO_Candidates.size() == 2)
        {
            break;
        }

        //ASSIGNING as ambiguous FIELDOBJECT:

        //MAKE AN AMBIGUOUS OBJECT:
        AddAmbiguousGoalPost(&(*it), AllObjects, vision);

        //debug << "Amb Object Visibility: "<< AllObjects->ambiguousFieldObjects.back().isObjectVisible() << ","<< vision->m_timestamp;
        ++it;
        //debug << "Distance to Goal[" << 0 <<"]: "<< FinalDistance << endl;

    }

}
