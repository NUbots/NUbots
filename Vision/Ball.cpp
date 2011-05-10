#include "Ball.h"
#include "Vision.h"
#include "ClassificationColours.h"
#include "TransitionSegment.h"
#include "ScanLine.h"
#include "ClassifiedSection.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Kinematics/Kinematics.h"

#if TARGET_OS_IS_WINDOWS
    #include <QDebug>
#endif
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
    ObjectCandidate largestCandidate;
    int sizeOfLargestCandidate = 0;
    Circle result;
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

        if(isObjectTooBig(PossibleBall, vision)) continue;

        //! Check if the ratio is correct: Height and Width ratio should be 1 as it is a circle,
        //! through can be skewed (camera moving), so we better put some threshold on it.
        if(!isCorrectCheckRatio(PossibleBall, height, width)) continue;


        //if(isObjectInRobot(PossibleBall, AllObjects)) continue;
        //debug << "BALL::FindBall  Possible Ball Found ";

        int sizeOfCandidate = (PossibleBall.getBottomRight().y - PossibleBall.getTopLeft().y); //Uses the height of the candidate, as width can be 0

        if(sizeOfCandidate > sizeOfLargestCandidate)
        {

            sizeOfLargestCandidate = sizeOfCandidate;
            largestCandidate = PossibleBall;
        }

    }
    //! Closely Classify the candidate: to obtain more information about the object (using closely classify function in vision)
    if(sizeOfLargestCandidate > 0)
    {
        float pinkPercentage = 0.0;
        largestCandidate.setColour(ClassIndex::orange);
        std::vector < Vector2<int> > ballPoints = classifyBallClosely(largestCandidate, vision,height, width, pinkPercentage);

        //! Perform Circle Fit: Must pass a threshold on fit to become a circle!
        //debug << "BALL::FindBall  Circle Fit ";

        Circle circle= isCorrectFit(ballPoints,largestCandidate, vision, pinkPercentage);
        if(circle.isDefined)
        {
            bool DistanceIsOK = isVisualBallDistanceCloseDistanceToPoint(circle, vision, largestCandidate, AllObjects);
            if(DistanceIsOK)
            {
                result = circle;
            }
        }
    }
    return result;
}
bool Ball::isVisualBallDistanceCloseDistanceToPoint(Circle circ, Vision* vision, const ObjectCandidate &PossibleBall, FieldObjects* AllObjects)
{
    bool result = false;

    //GET RADIUS FROM CIRCLE, CHANGE into a distance:
    double ballDistanceFactor=vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()*ORANGE_BALL_DIAMETER;
    float BALL_OFFSET = 0;
    float VisualDistance = (float)(ballDistanceFactor/(2*circ.radius)+BALL_OFFSET);
    float bearing = vision->CalculateBearing(circ.centreX);
    float elevation = vision->CalculateElevation(circ.centreY);
    float VisualFlatDistance = 0.0;
    Vector3< float > visualSphericalPosition;
    Vector3< float > transformedSphericalPosition;
    visualSphericalPosition[0] = VisualDistance;
    visualSphericalPosition[1] = bearing;
    visualSphericalPosition[2] = elevation;

    vector<float> ctvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraTransform, ctvector);
    if(isOK == true)
    {
        Matrix cameraTransform = Matrix4x4fromVector(ctvector);
        transformedSphericalPosition = Kinematics::TransformPosition(cameraTransform,visualSphericalPosition);
        VisualFlatDistance = transformedSphericalPosition[0];
    }

    //GET CENTRE POINT on CIRCLE, use Distance to point:

    vector<float> ctgvector;
    float distanceD2P =0.0;
    isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    if(isOK == true)
    {
        Matrix camera2groundTransform = Matrix4x4fromVector(ctgvector);
        Vector3<float> relativePoint = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
        distanceD2P = relativePoint[0];
        //#if DEBUG_VISION_VERBOSITY > 6
        //    debug << "\t\tCalculated Distance to Point: " << *distance<<endl;
        //#endif
    }
    else
    {
        return (!isObjectInRobot(PossibleBall, AllObjects));
    }

    //COMPARE: VisualDistance and Distance to Point:

    #if TARGET_OS_IS_WINDOWS
        qDebug() << "Distance Check: "<< "Measured:"<<VisualFlatDistance << "D2P:"<<distanceD2P;
    #endif
    //qDebug() << VisualFlatDistance << distanceD2P << elevation << 3.14/2-headElevation<<VisualDistance;
    float DistanceDifference = fabs(VisualFlatDistance - distanceD2P);

    if(isObjectInRobot(PossibleBall, AllObjects))
    {
        if(DistanceDifference > distanceD2P/5)
        {
            result = false;
        }
        else
        {
            result = true;
        }
    }
    else
    {
        if((DistanceDifference > distanceD2P*0.75) && (VisualFlatDistance > 40))
        {
            result = false;
        }
        else
        {
            result = true;
        }
    }



    return result;

}

bool Ball::isObjectAPossibleBall(const ObjectCandidate &PossibleBall)
{
    if(PossibleBall.getColour()== ClassIndex::orange ||
       PossibleBall.getColour()== ClassIndex::pink_orange ||
       PossibleBall.getColour() == ClassIndex::yellow_orange)
    {
        std::vector<TransitionSegment >segments = PossibleBall.getSegments();
        int orangeSize = 0;
        //int pinkSize = 0;
        for(unsigned int i = 0; i <segments.size(); i++)
        {
            /*if(segments[i].getColour() == ClassIndex::pink || segments[i].getColour() == ClassIndex::pink_orange)
            {
                pinkSize = pinkSize + segments[j].getSize();
            }*/
            if(segments[i].getColour() == ClassIndex::orange)
            {
                orangeSize = orangeSize + segments[i].getSize();
            }

        }
        if(orangeSize > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else{
        return false;
    }

}
bool Ball::isObjectTooBig(const ObjectCandidate &PossibleBall, Vision* vision)
{
    float MaxPixels = getMaxPixelsOfBall(vision);
    int heightOfPossibleBall = PossibleBall.getBottomRight().y - PossibleBall.getTopLeft().y;
    int widthOfPossibleBall = PossibleBall.getBottomRight().x - PossibleBall.getTopLeft().x;

    if(heightOfPossibleBall > MaxPixels || widthOfPossibleBall > MaxPixels)
    {
        return true;
    }
    else
    {
        return false;
    }
}
float Ball::getMaxPixelsOfBall(Vision* vision)
{
    float ClosestDistance = 30.0; //CM with Robot sitting down (40cm), standing about 50cm.
    float MaxPixels = vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS() * ORANGE_BALL_DIAMETER / ClosestDistance;
    return MaxPixels;
}

bool Ball::isObjectInRobot(const ObjectCandidate &PossibleBall, FieldObjects *AllObjects)
{
    Vector2<int> topLeft = PossibleBall.getTopLeft();
    Vector2<int> bottomRight = PossibleBall.getBottomRight();
    bool isInRobot = false;
    //! Check the Ambiguous Objects: for robots:
    for(unsigned int i = 0; i < AllObjects->ambiguousFieldObjects.size(); i++)
    {
        if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_PINK_ROBOT_UNKNOWN)
        {
            Vector2<int> robotTopLeft, robotBottomLeft;
            robotTopLeft.x = AllObjects->ambiguousFieldObjects[i].ScreenX() -  AllObjects->ambiguousFieldObjects[i].getObjectWidth()/2;
            robotTopLeft.y = AllObjects->ambiguousFieldObjects[i].ScreenY() -  AllObjects->ambiguousFieldObjects[i].getObjectHeight()/2;
            robotBottomLeft.x = AllObjects->ambiguousFieldObjects[i].ScreenX() +  AllObjects->ambiguousFieldObjects[i].getObjectWidth()/2;
            robotBottomLeft.y = AllObjects->ambiguousFieldObjects[i].ScreenY() +  AllObjects->ambiguousFieldObjects[i].getObjectHeight()/2;

            if( topLeft.x >= robotTopLeft.x && topLeft.y >= robotTopLeft.y && bottomRight.x <= robotBottomLeft.x &&  bottomRight.y <= robotBottomLeft.y)
            {
                if( (bottomRight.y - topLeft.y)  <  (robotBottomLeft.y - robotTopLeft.y)/10)
                {
                    isInRobot = true;
                    return isInRobot;
                }
            }
        }
    }
    return isInRobot;
}

std::vector < Vector2<int> > Ball::classifyBallClosely(const ObjectCandidate &PossibleBall,Vision* vision,int height, int width, float &pinkPercentage)
{

    int buffer = 40; //Stop points close to the screen from been included to as a ball point.
    int pinkCount = 0;
    Vector2<int> TopLeft = PossibleBall.getTopLeft();
    Vector2<int> BottomRight = PossibleBall.getBottomRight();
    int midX =  (int)((BottomRight.x-TopLeft.x)/2)+TopLeft.x;
    int midY =  (int)((BottomRight.y-TopLeft.y)/2)+TopLeft.y;
    Vector2<int> SegStart;
    SegStart.x = midX;
    SegStart.y = TopLeft.y-((int)(BottomRight.y-TopLeft.y)/4);
    Vector2<int> SegEnd;
    SegEnd.x = midX;
    SegEnd.y = BottomRight.y+(int)((BottomRight.y-TopLeft.y)/4);
    TransitionSegment tempSeg(SegStart,SegEnd,ClassIndex::unclassified,PossibleBall.getColour(),ClassIndex::unclassified);
    //qDebug() << "Possible Ball Colour: "<<ClassIndex::getColourNameFromIndex(PossibleBall.getColour());
    ScanLine tempLine = ScanLine();

    //! Maximum ball points = 4*2 = 8;
    int MAX_BALL_POINTS_PER_DIECTION = 15;
    int spacings = (int)(BottomRight.y - TopLeft.y)/MAX_BALL_POINTS_PER_DIECTION;
    if(spacings < 2)
    {
        spacings = 2;
    }
    //qDebug() << spacings ;
    std::vector<unsigned char> colourlist;
    colourlist.reserve(3);
    colourlist.push_back(ClassIndex::orange);
    colourlist.push_back(ClassIndex::pink_orange);
    colourlist.push_back(ClassIndex::yellow_orange);
    int direction = ScanLine::DOWN;
    //qDebug() << "Horizontal Scan : ";
    int bufferSize = (int)((BottomRight.y-TopLeft.y)/8);
    vision->CloselyClassifyScanline(&tempLine,&tempSeg,spacings, direction,colourlist,bufferSize);

    std::vector< Vector2<int> > BallPoints,LeftBallPoints,RightBallPoints,TopBallPoints,BottomBallPoints,GoodBallPoints;

    BallPoints.push_back(SegStart);
    BallPoints.push_back(SegEnd);
    //! Debug Output for small scans:
    for(int i = 0; i < tempLine.getNumberOfSegments(); i++)
    {
        TransitionSegment* tempSegement = tempLine.getSegment(i);
        if(tempSegement->getSize() <= 2) continue;
        //! Check if the segments are at the edge of screen
        if(!(tempSegement->getStartPoint().x < buffer  || tempSegement->getStartPoint().y < buffer))
        {

            if(tempSegement->getBeforeColour() == ClassIndex::green || tempSegement->getBeforeColour() == ClassIndex::shadow_object || tempSegement->getBeforeColour() == ClassIndex::unclassified)
            {
                //qDebug() << "Pushing LEFT" << LeftBallPoints.size();
                LeftBallPoints.push_back(tempSegement->getStartPoint());
                GoodBallPoints.push_back(tempSegement->getStartPoint());
            }
            if(tempSegement->getBeforeColour() == ClassIndex::pink) pinkCount++;
            BallPoints.push_back(tempSegement->getStartPoint());

        }
        if(!(tempSegement->getEndPoint().x >= width-buffer || tempSegement->getEndPoint().y >= height-buffer))
        {
            if(tempSegement->getAfterColour() == ClassIndex::green || tempSegement->getAfterColour() == ClassIndex::shadow_object || tempSegement->getAfterColour() == ClassIndex::unclassified)
            {
                //qDebug() << "Pushing RIGHT" << RightBallPoints.size();
                RightBallPoints.push_back(tempSegement->getEndPoint());
                GoodBallPoints.push_back(tempSegement->getStartPoint());
            }
            if(tempSegement->getAfterColour() == ClassIndex::pink) pinkCount++;
            BallPoints.push_back(tempSegement->getEndPoint());
        }

        /*qDebug() << "Horizontal Points At " <<i<<"\t Size: "<< tempSegement->getSize()<< "\t Start(x,y),End(x,y):("<< tempSegement->getStartPoint().x
                <<","<< tempSegement->getStartPoint().y << ")("<< tempSegement->getEndPoint().x
                <<","<< tempSegement->getEndPoint().y << ")" <<  ClassIndex::getColourNameFromIndex(tempSegement->getBeforeColour()) << ClassIndex::getColourNameFromIndex(tempSegement->getAfterColour());
        */
    }

    SegStart.x = TopLeft.x-((int)(BottomRight.x-TopLeft.x)/4);
    SegStart.y = midY;
    SegEnd.x = BottomRight.x+((int)(BottomRight.x-TopLeft.x)/4);
    SegEnd.y = midY;
    tempSeg = TransitionSegment(SegStart,SegEnd,ClassIndex::unclassified,PossibleBall.getColour(),ClassIndex::unclassified);
    tempLine = ScanLine();

    BallPoints.push_back(SegStart);
    BallPoints.push_back(SegEnd);

    //! Maximum ball points = 4*2 = 8;
    spacings = (int)(BottomRight.x - TopLeft.x)/MAX_BALL_POINTS_PER_DIECTION;
    if(spacings < 2)
    {
        spacings = 2;
    }
    //qDebug() << "Vertical Scan : ";
    direction = ScanLine::LEFT;
    bufferSize = (int)((BottomRight.x-TopLeft.x)/8);
    vision->CloselyClassifyScanline(&tempLine,&tempSeg,spacings, direction, colourlist,bufferSize);
    for(int i = 0; i < tempLine.getNumberOfSegments(); i++)
    {

        TransitionSegment* tempSegement = tempLine.getSegment(i);
        if(tempSegement->getSize() <= 2) continue;
        //! Check if the segments are at the edge of screen
        if(!(tempSegement->getStartPoint().x < buffer || tempSegement->getStartPoint().y < buffer))
        {
            if(tempSegement->getBeforeColour() == ClassIndex::green || tempSegement->getBeforeColour() == ClassIndex::shadow_object || tempSegement->getBeforeColour() == ClassIndex::unclassified)
            {
                //qDebug() << "Pushing TOP" << TopBallPoints.size();
                TopBallPoints.push_back(tempSegement->getStartPoint());
                GoodBallPoints.push_back(tempSegement->getStartPoint());
            }
            if(tempSegement->getAfterColour() == ClassIndex::pink) pinkCount++;
            BallPoints.push_back(tempSegement->getStartPoint());
        }

        float headElevation = 0.0;
        vision->getSensorsData()->getPosition(NUSensorsData::HeadPitch,headElevation);

        if(!(tempSegement->getEndPoint().y >= height-buffer || tempSegement->getEndPoint().x >= width-buffer) &&  headElevation < 0.3)
        {
            if(tempSegement->getAfterColour() == ClassIndex::green || tempSegement->getAfterColour() == ClassIndex::shadow_object || tempSegement->getAfterColour() == ClassIndex::unclassified)
            {
                //qDebug() << "Pushing BOTTOM" << BottomBallPoints.size();
                BottomBallPoints.push_back(tempSegement->getEndPoint());
                GoodBallPoints.push_back(tempSegement->getStartPoint());
            }
            if(tempSegement->getAfterColour() == ClassIndex::pink) pinkCount++;
            BallPoints.push_back(tempSegement->getEndPoint());
        }
        //vision->getSensorsData()->getJointPosition(NUSensorsData::HeadPitch,headElevation);
        //qDebug() << "Ball Head Elevation:" << headElevation;
        /*qDebug() << "Veritcal Points At " <<i<<"\t Size: "<< tempSegement->getSize()<< "\t Start(x,y),End(x,y):("<< tempSegement->getStartPoint().x
                <<","<< tempSegement->getStartPoint().y << ")("<< tempSegement->getEndPoint().x
                <<","<< tempSegement->getEndPoint().y << ")" <<  ClassIndex::getColourNameFromIndex(tempSegement->getBeforeColour()) << ClassIndex::getColourNameFromIndex(tempSegement->getAfterColour());
        */
    }

    //FIND THE LARGEST SIDE WITH GOOD POINTS and RETURN IT:
    std::vector < Vector2<int> > finalPoints = TopBallPoints;
    if( LeftBallPoints.size() > finalPoints.size() && !(LeftBallPoints.empty()))
    {
        //qDebug() << "Using LeftBallPoints: " << finalPoints.size() << LeftBallPoints.size() ;
        finalPoints = LeftBallPoints;

    }
    if( RightBallPoints.size() > finalPoints.size() && !(RightBallPoints.empty()))
    {
        //qDebug() << "Using RightBallPoints: " << finalPoints.size() << RightBallPoints.size();
        finalPoints = RightBallPoints;
    }
    if( BottomBallPoints.size() > finalPoints.size() && !(BottomBallPoints.empty()))
    {
        //qDebug() << "Using BottomBallPoints: " << finalPoints.size() << BottomBallPoints.size();
        finalPoints = BottomBallPoints;
    }
    //Ball is BIG but surrounded by white:
    if(finalPoints.size() < MAX_BALL_POINTS_PER_DIECTION*0.8 && (PossibleBall.width() > 30 || PossibleBall.height() > 30) )
    {
        //qDebug() << "Using BallPoints: " << finalPoints.size() << BallPoints.size();
        finalPoints = GoodBallPoints;
    }

    #if TARGET_OS_IS_WINDOWS
        qDebug() << "Pink Count: "<< pinkCount << BallPoints.size() << (pinkCount*1.0/BallPoints.size())*100 << "%";
    #endif
    pinkPercentage = (pinkCount*1.0/BallPoints.size())*100;
    return finalPoints;

}
bool Ball::isCorrectCheckRatio(const ObjectCandidate &PossibleBall,int height, int width)
{
    //debug << "Checking Ratio: " << PossibleBall.aspect();

    //! Check if at Edge of Screen, if so continue with other checks, otherwise, look at ratio and check if in thresshold
    int boarder = 10; //! Boarder of pixels
    if (( PossibleBall.getBottomRight().y - PossibleBall.getTopLeft().y) <= 3) return false;
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
Circle Ball::isCorrectFit(const std::vector < Vector2<int> > &ballPoints, const ObjectCandidate &PossibleBall, Vision* vision, float &pinkPercentage)
{
    Circle circ;
    circ.radius = 0.0;
    circ.isDefined = false;
    CircleFitting CircleFit;

    //debug << "Points:";
    #if TARGET_OS_IS_WINDOWS
    for(unsigned int i =0; i < ballPoints.size(); i++)
    {
        qDebug() <<ballPoints[i].x << ","<<ballPoints[i].y;
    }
    #endif

    if(ballPoints.size() >= 5)
    {

            circ = CircleFit.FitCircleLMF(ballPoints);
            if(circ.sd > 3.5 ||  circ.radius*2 > getMaxPixelsOfBall(vision) )
            {
                circ.isDefined = false;
            }
            #if TARGET_OS_IS_WINDOWS
            qDebug() << "Circle found " << circ.isDefined<<": (" << circ.centreX << "," << circ.centreY << ") Radius: "<< circ.radius << " Fitting: " << circ.sd<< endl;
            #endif
    }
    //SMALL BALLS:
    else if ((PossibleBall.width() < 20 || PossibleBall.height() < 20) && pinkPercentage < 40) //(ballPoints.size() < 7) &&
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
