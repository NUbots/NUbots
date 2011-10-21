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
    m_ball_colours.push_back(ClassIndex::orange);
    m_ball_colours.push_back(ClassIndex::pink_orange);
    m_ball_colours.push_back(ClassIndex::yellow_orange);
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
		#if DEBUG_VISION_VERBOSITY > 7        
		debug << "BALL::FindBall  Possible Ball Found ";
		#endif
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

        Circle circle = isCorrectFit(ballPoints,largestCandidate, vision, pinkPercentage);
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
    float distanceD2P = 0.0;
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

    //#if TARGET_OS_IS_WINDOWS
    //    qDebug() << "Distance Check: "<< "Measured:"<<VisualFlatDistance << "D2P:"<<distanceD2P;
    //#endif
    //qDebug() << VisualFlatDistance << distanceD2P << elevation << 3.14/2-headElevation<<VisualDistance;
    float DistanceDifference = fabs(VisualFlatDistance - distanceD2P);

    return distanceD2P < 1.1*VisualFlatDistance;
}

bool Ball::isObjectAPossibleBall(const ObjectCandidate &PossibleBall)
{
    if(PossibleBall.getColour()== ClassIndex::orange || PossibleBall.getColour()== ClassIndex::pink_orange || PossibleBall.getColour() == ClassIndex::yellow_orange)
    {
        std::vector<TransitionSegment >segments = PossibleBall.getSegments();
        int orangeSize = 0;
        int pinkSize = 0;
        int yellowSize = 0;
        for(unsigned int i = 0; i <segments.size(); i++)
        {
            if (segments[i].getColour() == ClassIndex::pink || segments[i].getColour() == ClassIndex::pink_orange)
                pinkSize = pinkSize + segments[i].getSize();
            if (segments[i].getColour() == ClassIndex::orange)
                orangeSize = orangeSize + segments[i].getSize();
            if (segments[i].getColour() == ClassIndex::yellow || segments[i].getColour() == ClassIndex::yellow_orange)
                yellowSize = yellowSize + segments[i].getSize();

        }
        
        if(orangeSize > 0 and orangeSize > pinkSize and orangeSize > yellowSize)
            return true;
        else
            return false;
    }
    else
        return false;
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
    float MaxPixels = vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS() * ORANGE_BALL_DIAMETER / 2*ClosestDistance;
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

/*! @brief A potential ball has been found, now we closely classify inside the bounding box, so that we can fit a circle
    @param PossibleBall the potential ball
    @param vision a pointer to the vision module
    @param height the height of the image in pixels
    @param width the width of the image in pixels
    @param pinkPercentage a Wong hack to check the amount of pink inside this candidate
 
    @return a list of Vector2 for each point to be used in a circle fit
 */
std::vector < Vector2<int> > Ball::classifyBallClosely(ObjectCandidate &PossibleBall, Vision* vision, int height, int width, float &pinkPercentage)
{
    /* The plan is to decide which way to scan based on where the possible ball is in the image
            if the possible ball on the bottom and we are looking down, then scan from the top (only)
            else if the possible ball on the left, then scan from the right (only)
            else if the possible ball on the right, then scan from the left (only)
            else scan from both left and right
     */
    
    // The only thing we want out of the PossibleBall is the bounding box.
    int horziontalscanspacing = vision->getScanSpacings();
    int verticalscanscaping = 4;
    Vector2<int> TopLeft = PossibleBall.getTopLeft();
    TopLeft.x -= horziontalscanspacing;
    if (TopLeft.x < 0) TopLeft.x = 0;
    TopLeft.y -= verticalscanscaping;
    if (TopLeft.y < 0) TopLeft.y = 0;
    Vector2<int> BottomRight = PossibleBall.getBottomRight();
    BottomRight.x += horziontalscanspacing;
    if (BottomRight.x >= width) BottomRight.x = width;
    BottomRight.y += verticalscanscaping;
    if (BottomRight.y >= height) BottomRight.y = height;
    
    // Now check the distances from the left, right, top and bottom of the image
    float pixelsfromtop = TopLeft.y;
    float pixelsfrombottom = height - BottomRight.y;
    float pixelsfromleft = TopLeft.x;
    float pixelsfromright = width - BottomRight.x;
    float headpitch = 0;
    vision->getSensorsData()->getPosition(NUSensorsData::HeadPitch, headpitch);

    std::vector < Vector2<int> > finalPoints;
    int numscans = 0;
    if (headpitch > 0.25 and pixelsfrombottom < 60)
    {   // if we are looking down, so far its likely the ball will be occluded by the knees or chest; scan from the top down
        std::vector < Vector2<int> > points = scanDown(TopLeft, BottomRight, vision);
        finalPoints.insert(finalPoints.end(), points.begin(), points.end());
        numscans = BottomRight.x - TopLeft.x;
    }
    else if (pixelsfromleft < 4)
    {
        std::vector < Vector2<int> > points = scanLeft(TopLeft, BottomRight, vision);
        finalPoints.insert(finalPoints.end(), points.begin(), points.end());
        numscans = BottomRight.y - TopLeft.y;
    }
    else if (pixelsfromright < 4)
    {
        std::vector < Vector2<int> > points = scanRight(TopLeft, BottomRight, vision);
        finalPoints.insert(finalPoints.end(), points.begin(), points.end());
        numscans = BottomRight.y - TopLeft.y;
    }
    else
    {
        std::vector < Vector2<int> > points = scanLeft(TopLeft, BottomRight, vision);
        finalPoints.insert(finalPoints.end(), points.begin(), points.end());
        points = scanRight(TopLeft, BottomRight, vision);
        finalPoints.insert(finalPoints.end(), points.begin(), points.end());
        numscans = 2*(BottomRight.y - TopLeft.y);
    }
    
    // this is a hack too. I am going to update the size of the ObjectCandidate with the new closely classified points
    // then I am going to check the size, if its small then fitting a circle is pointless, so NO points will be returned
    
    // update the size of the possible ball
    Vector2<int> newtopleft(width, height);
    Vector2<int> newbottomright(0,0);
    for (size_t i=0; i<finalPoints.size(); i++)
    {
        Vector2<int>& p = finalPoints[i];
        if (p.x < newtopleft.x)
            newtopleft.x = p.x;
        if (p.x > newbottomright.x)
            newbottomright.x = p.x;
        if (p.y < newtopleft.y)
            newtopleft.y = p.y;
        if (p.y > newbottomright.y)
            newbottomright.y = p.y;
        
    }
    
    // this is a hack; this should really be done earlier, but we need to throw out balls which are too small, or ones which are not very filled in
    if (finalPoints.size() < 4 or finalPoints.size() < 0.3*numscans)
        PossibleBall.setBottomRight(PossibleBall.getTopLeft());
    else
    {
        PossibleBall.setTopLeft(newtopleft);
        PossibleBall.setBottomRight(newbottomright);
    }
    
    // if the updated size is small, throw away the points
    if (PossibleBall.width()*PossibleBall.height() < 121)
        return std::vector < Vector2<int> >();
    else
        return finalPoints;
}

std::vector < Vector2<int> > Ball::scanLeft(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision)
{
    std::vector < Vector2<int> > points;
    Vector2<int> current_point;
    for (int i=topleft.y; i<bottomright.y; i++)
    {
        current_point.y = i;
        current_point.x = -1;
        int previouscolour = ClassIndex::unclassified;
        for (int j=bottomright.x; j>topleft.x; j--)
        {
            int colour = vision->classifyPixel(j,i);
            if (vision->isValidColour(colour, m_ball_colours) and vision->isValidColour(previouscolour, m_ball_colours) and current_point.x < 0)
                current_point.x = j-1;
            if (colour == ClassIndex::orange and current_point.x >= 0)
            {
                points.push_back(current_point);
                break;
            }
            previouscolour = colour;
        }
    }
    return points;
}

std::vector < Vector2<int> > Ball::scanRight(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision)
{
    std::vector < Vector2<int> > points;
    Vector2<int> current_point;
    for (int i=topleft.y; i<bottomright.y; i++)
    {
        current_point.y = i;
        current_point.x = -1;
        int previouscolour = ClassIndex::unclassified;
        for (int j=topleft.x; j<bottomright.x; j++)
        {
            int colour = vision->classifyPixel(j,i);
            if (vision->isValidColour(colour, m_ball_colours) and vision->isValidColour(previouscolour, m_ball_colours) and current_point.x < 0)
                current_point.x = j-1;
            if (colour == ClassIndex::orange and current_point.x >= 0)
            {
                points.push_back(current_point);
                break;
            }
            previouscolour = colour;
        }
    }
    return points;
}

std::vector < Vector2<int> > Ball::scanDown(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision)
{
    std::vector < Vector2<int> > points;
    Vector2<int> current_point;
    for (int i=topleft.x; i<bottomright.x; i++)
    {
        current_point.x = i;
        current_point.y = -1;
        int previouscolour = ClassIndex::unclassified;
        for (int j=topleft.y; j<bottomright.y; j++)
        {
            int colour = vision->classifyPixel(i,j);
            if (vision->isValidColour(colour, m_ball_colours) and vision->isValidColour(previouscolour, m_ball_colours) and current_point.y < 0)
                current_point.y = j-1;
            if (colour == ClassIndex::orange and current_point.y >= 0)
            {
                points.push_back(current_point);
                break;
            }
            previouscolour = colour;
        }
            
    }
    return points;
}

std::vector < Vector2<int> > Ball::scanUp(const Vector2<int>& topleft, const Vector2<int>& bottomright, Vision* vision)
{
    std::vector < Vector2<int> > points;
    Vector2<int> current_point;
    int previouscolour = ClassIndex::unclassified;
    for (int i=topleft.x; i<bottomright.x; i++)
    {
        current_point.x = i;
        current_point.y = -1;
        for (int j=bottomright.y; j>topleft.y; j--)
        {
            int colour = vision->classifyPixel(i,j);
            if (vision->isValidColour(colour, m_ball_colours) and vision->isValidColour(previouscolour, m_ball_colours) and current_point.y < 0)
                current_point.y = j-1;
            if (colour == ClassIndex::orange and current_point.y >= 0)
            {
                points.push_back(current_point);
                break;
            }
            previouscolour = colour;
        }
        
    }
    return points;
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

    if (PossibleBall.height() < 2 or PossibleBall.width() < 2)
        circ.isDefined = false;
    else if(ballPoints.size() > 0)   
    {   // we can measure the distance to the ball visually, so use a circle fit to measure the radius
        circ = CircleFit.FitCircleLMF(ballPoints);
        if(circ.sd > 3.5 ||  circ.radius*2 > getMaxPixelsOfBall(vision) )
        {   // can't really throw out much based on the circle fit, this is a VERY loose check
            circ.isDefined = false;
        }
        //#if TARGET_OS_IS_WINDOWS
        //    qDebug() << "Circle found " << circ.isDefined<<": (" << circ.centreX << "," << circ.centreY << ") Radius: "<< circ.radius << " Fitting: " << circ.sd/circ.radius << endl;
        //#endif
    }
    else
    {   // balls that are too small to measure their distance visually
        Vector2<int> bottomRight = PossibleBall.getBottomRight();
        Vector2<int> topLeft = PossibleBall.getTopLeft();
        
        // the centre of the object candiate becomes the centre of the ball
        circ.centreX = (bottomRight.x + topLeft.x)/2;
        circ.centreY = (bottomRight.y + topLeft.y)/2;
        circ.isDefined = true;
        circ.sd = fabs(fabs(bottomRight.x - topLeft.x) - fabs(bottomRight.y - topLeft.y));      //!< Uncertianty, is somewhere between the candidates height and widths
        circ.radius = (PossibleBall.height() + PossibleBall.width())/2;
        
        float bearing = vision->CalculateBearing(circ.centreX);
        float elevation = vision->CalculateElevation(circ.centreY);
        float distance;
        // check if distance to point is available
        vector<float> ctgvector;
        bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
        if(isOK == true)
        {   // if distance to point is available, then hack it, and set the radius of the ball to be what it should be
            Matrix camera2groundTransform = Matrix4x4fromVector(ctgvector);
            Vector3<float> relativePoint = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);
            distance = relativePoint[0];
            double ballDistanceFactor = vision->EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()*ORANGE_BALL_DIAMETER;
            circ.radius = ballDistanceFactor/(2*distance);
        }
    }

    //debug << "BALL::CircleFit returning circle r =" << circ.radius;
    //delete CircleFit;
    return circ;
}
