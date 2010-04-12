#include "Ball.h"
#include "ClassificationColours.h"
#include "TransitionSegment.h"
#include "ScanLine.h"
#include "ClassifiedSection.h"
#include "debug.h"
#include "debugverbosityvision.h"

Ball::Ball()
{
    //debug<< "Vision::DetectBall : Ball Class created" << endl;
}
Ball::~Ball()
{
}

//! Finds the ball segments and groups updates the ball in fieldObjects (Vision is used to further classify the object)
Circle Ball::FindBall(std::vector <ObjectCandidate> FO_Candidates, FieldObjects* AllObjects, Vision* vision,int height,int width)
{
    Circle result;
    Circle tempresult;
    result.centreX = 0;
    result.centreY = 0;
    result.radius = 0;
    result.sd = 0;

    //! Go through all the candidates: to find a possible ball
    //debug <<"FO_Candidates.size():"<< FO_Candidates.size();
    for(unsigned int i = 0; i  < FO_Candidates.size(); i++)
    {

        ObjectCandidate PossibleBall = FO_Candidates[i];

        if(!isObjectAPossibleBall(PossibleBall)) continue;

        //! Check if the ratio is correct: Height and Width ratio should be 1 as it is a circle,
        //! through can be skewed (camera moving), so we better put some threshold on it.
        if(!isCorrectCheckRatio(PossibleBall, height, width)) continue;
        //debug << "BALL::FindBall  Possible Ball Found ";

        //! Closely Classify the candidate: to obtain more information about the object (using closely classify function in vision)
        std::vector < Vector2<int> > ballPoints = classifyBallClosely(PossibleBall, vision,height, width);

        //! Perform Circle Fit: Must pass a threshold on fit to become a circle!
        //debug << "BALL::FindBall  Circle Fit ";

        tempresult = isCorrectFit(ballPoints,PossibleBall);

        //debug << "BALL::FindBall  Circle Fit finnsihed";
        if (tempresult.radius  > result.radius)
        {
            //debug << "BALL::FindBall  Updated with larger circle"<< endl;
            result = tempresult;
        }
        //! Use Circle Fit information to update the FieldObjects

        //debug << "BALL::FindBall  Circle Fit finnsihed"<<endl;
        //! check if current object is larger then object before.
    }

    return result;
}

bool Ball::isObjectAPossibleBall(ObjectCandidate PossibleBall)
{
    if(PossibleBall.getColour()== ClassIndex::orange ||
       PossibleBall.getColour()== ClassIndex::red_orange ||
       PossibleBall.getColour() == ClassIndex::yellow_orange)
    {
        return true;
    }
    else{
        return false;
    }
}
std::vector < Vector2<int> > Ball::classifyBallClosely(ObjectCandidate PossibleBall,Vision* vision,int heigth, int width)
{
    Vector2<int> TopLeft = PossibleBall.getTopLeft();
    Vector2<int> BottomRight = PossibleBall.getBottomRight();
    int midX =  (int)((BottomRight.x-TopLeft.x)/2)+TopLeft.x;
    Vector2<int> SegStart;
    SegStart.x = midX;
    SegStart.y = TopLeft.y;
    Vector2<int> SegEnd;
    SegEnd.x = midX;
    SegEnd.y = BottomRight.y;
    TransitionSegment tempSeg(SegStart,SegEnd,ClassIndex::unclassified,PossibleBall.getColour(),ClassIndex::unclassified);
    ScanLine tempLine;

    int spacings = 2;
    int direction = ClassifiedSection::DOWN;
    vision->CloselyClassifyScanline(&tempLine,&tempSeg,spacings, direction);

    std::vector< Vector2<int> > BallPoints;


    //! Debug Output for small scans:
    for(int i = 0; i < tempLine.getNumberOfSegments(); i++)
    {
        TransitionSegment* tempSegement = tempLine.getSegment(i);
        //! Check if the segments are at the edge of screen
        if(!(tempSegement->getStartPoint().x < 0 || tempSegement->getStartPoint().y < 0))
        {
            BallPoints.push_back(tempSegement->getStartPoint());
        }
        if(!(tempSegement->getEndPoint().x >= heigth-1 || tempSegement->getEndPoint().x >= width-1))
        {
            BallPoints.push_back(tempSegement->getEndPoint());
        }

        /* debug << "At " <<i<<"\t Size: "<< tempSeg->getSize()<< "\t Start(x,y),End(x,y):("<< tempSeg->getStartPoint().x
                <<","<< tempSeg->getStartPoint().y << ")("<< tempSeg->getEndPoint().x
                <<","<< tempSeg->getEndPoint().y << ")";*/

    }

    return BallPoints;

}
bool Ball::isCorrectCheckRatio(ObjectCandidate PossibleBall,int height, int width)
{
    //debug << "Checking Ratio: " << PossibleBall.aspect();

    //! Check if at Edge of Screen, if so continue with other checks, otherwise, look at ratio and check if in thresshold
    int boarder = 10; //! Boarder of pixels

    if (PossibleBall.getBottomRight().x <= width-boarder &&
        PossibleBall.getBottomRight().y <= height-boarder &&
        PossibleBall.getTopLeft().x >=0+boarder  &&
        PossibleBall.getTopLeft().y >=0+boarder  )
    {
        //POSSIBLE BALLS ARE:
        //      Objects which have grouped segments,
        //      or objects with one segment, but very small (still like to consider).
        if((PossibleBall.aspect() > 0.3 && PossibleBall.aspect() < 2 )|| PossibleBall.aspect()==0)
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
Circle Ball::isCorrectFit(std::vector < Vector2<int> > ballPoints, ObjectCandidate PossibleBall)
{
    Circle circ;
    circ.radius = 0.0;
    circ.isDefined = false;
    CircleFitting CircleFit;

    //debug << "Points:";
   /* for(int i =0; i < ballPoints.size(); i++)
    {
        debug << "("<<ballPoints[i].x << ","<<ballPoints[i].y<< ")";
    }*/
    if(ballPoints.size() > 5)
    {

            circ = CircleFit.FitCircleLMA(ballPoints);
            //debug << "Circle found " << circ.isDefined<<": (" << circ.centreX << "," << circ.centreY << ") Radius: "<< circ.radius << " Fitting: " << circ.sd<< endl;

    }
    else
    {
        Vector2<int> bottomRight = PossibleBall.getBottomRight();
        Vector2<int> topLeft = PossibleBall.getTopLeft();
        //! find midPoints of the Candidate:
        circ.centreX = (bottomRight.x + topLeft.x)/2;
        circ.centreY = (bottomRight.y + topLeft.y)/2;
        circ.isDefined = true;
        circ.sd = fabs(fabs(bottomRight.x - topLeft.x) - fabs(bottomRight.y - topLeft.y));      //!< Uncertianty, is somewhere between the candidates height and widths
        //! Select the Largest side as radius:
        if(fabs(bottomRight.x - topLeft.x) > fabs(bottomRight.y - topLeft.y))
        {
             circ.radius= fabs(bottomRight.x - topLeft.x)/2;
        }
        else
        {
            circ.radius = fabs(bottomRight.y - topLeft.y)/2;
        }
        //debug << "Circle cannot be fitted: Used Candidate information" << endl;
    }

    //debug << "BALL::CircleFit returning circle r =" << circ.radius;
    //delete CircleFit;
    return circ;
}
