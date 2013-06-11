#include "LineDetection.h"
#include "TransitionSegment.h"
#include "ClassificationColours.h"
#include <math.h>
#define MIN_POINTS_ON_LINE_FINAL 5
#define MIN_POINTS_ON_LINE 4
#define MAX_DISTANCE_T_GOAL 40*2 //40cm away from goal with factor of 2 error
#define MAX_DISTANCE_L_GOAL 80*2 //80cm from point to goal with factor of 2 error
//#define LINE_SEARCH_GRID_SIZE 4
//#define POINT_SEARCH_GRID_SIZE 4
#include <stdio.h>

#include <vector>
#include "Vision.h"
#include "EllipseFit.h"
#include "fitellipsethroughcircle.h"
#include "Tools/Math/StlVector.h"
//For distance to Point:
#include "Infrastructure/NUBlackboard.h"
#include "../Kinematics/Kinematics.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "Tools/Profiling/Profiler.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include <ctime>

#if TARGET_OS_IS_WINDOWS
    #include <QDebug>
#endif

LineDetection::LineDetection(){

    //Reserving Space for Vector Elements
    linePoints.reserve(MAX_LINEPOINTS);
    fieldLines.reserve(MAX_FIELDLINES);
    cornerPoints.reserve(MAX_CORNERPOINTS);
    transformedFieldLines.reserve(MAX_FIELDLINES);
    TotalValidLines = 0;
}

LineDetection::~LineDetection(){
return;
}

void LineDetection::FormLines(FieldObjects* AllObjects, Vision* vision, NUSensorsData* data) {

    //Setting up the variables:
    int spacing = vision->getScanSpacings();
    LINE_SEARCH_GRID_SIZE = spacing/4; //Should be 4 at 320width

    int image_height = vision->getImageHeight();
    int image_width = vision->getImageWidth();
    sensorsData = data;

    //
    //LinePointCounter = 0;
    TotalValidLines = 0;
    //FieldLinesCounter = 0;
    //CornerPointCounter = 0;
    //FIND LINE POINTS:
    #if DEBUG_VISION_VERBOSITY > 5
        clock_t start, end;
        start = clock();
    #endif
    //update local pointer to sensors data:


    //FindLinePoints(scanArea,vision);


    //qDebug() << "Robot Segments: " << robotSegments.size();
    /*for(unsigned int j = 0; j < linePoints.size(); j++)
    {
         qDebug() << "Point " <<j << ":" << linePoints[j].x << "," << linePoints[j].y;
    }*/

    //
    //qDebug() << "Finding Lines with segments:  " << linePoints.size();

    //Profiler prof = Profiler("AARON");
    //prof.start();
    /*AARON'S OLD LINE DETECTION*/
    FindFieldLines(image_width,image_height);
    /*AARON'S OLD LINE DETECTION*/
    //prof.split("FindFieldLines");
    //debug << prof;
    //qDebug() << "Lines found: " << fieldLines.size()<< "\t" << "Vaild: "<< TotalValidLines;
    //for(unsigned int i = 0; i < fieldLines.size(); i++)
    //{
    //    if(fieldLines[i].valid)
    //    {
            ////qDebug()<<i << ": "<< fieldLines[i].leftPoint.x << "," << fieldLines[i].leftPoint.y << "\t"
                    //<< fieldLines[i].rightPoint.x << "," << fieldLines[i].rightPoint.y
                    //<< "\t Points: " << fieldLines[i].numPoints;
    //        std::vector<LinePoint*> points = fieldLines[i].getPoints();
    //        for( int j = 0; j < fieldLines[i].numPoints ; j++)
    //        {

                ////qDebug() << "Point " <<j << ":" << points[j]->x << "," << points[j]->y;
    //        }

    //    }
    //}

    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Pre-FindPenaltySpot" << std::endl;
    #endif
    
    //qDebug() << "Finding Penalty Spots:";
    //FindPenaltySpot(vision);
    //qDebug() << "Finding Corner Points:";
    
    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Post-FindPenaltySpot" << std::endl;
    #endif


    TransformLinesToWorldModelSpace(vision);


    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Pre-FindCornerPoints" << std::endl;
    #endif
    
    FindCornerPoints(image_width,image_height,vision);
    
    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Post-FindCornerPoints: Found " << cornerPoints.size() << " corners."<< std::endl;
    #endif
    //qDebug() << "Corners found: " << cornerPoints.size();
    //for (unsigned int i = 0; i < cornerPoints.size(); i ++)
    //{
    //    if(cornerPoints[i].CornerType == 0) //L
    //    {
            //qDebug() << "L Corner at " << cornerPoints[i].PosX << "," << cornerPoints[i].PosY <<
            //        "\t Orientation, Direction: " << cornerPoints[i].Orientation << "," << cornerPoints[i].Direction;
    //    }
    //    else
    //    {
            //qDebug() << "T Corner at " << cornerPoints[i].PosX << "," << cornerPoints[i].PosY <<
            //        "\t Orientation, Direction: " << cornerPoints[i].Orientation << "," << cornerPoints[i].Direction;
    //    }
    //}
    #if DEBUG_VISION_VERBOSITY > 5
    clock_t startCorner = clock();
    #endif
    //qDebug() << "Decode Corners:";
    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Pre-DecodeCorners" << std::endl;
    #endif
    
    DecodeCorners(AllObjects, vision->m_timestamp, vision);
    
    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Post-DecodeCorners" << std::endl;
    #endif
    //qDebug() << "Decode Penalty Spots:";
    
    
    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Pre-DecodePenaltySpot" << std::endl;
    #endif
    
    DecodePenaltySpot(AllObjects, vision->m_timestamp);
    
    #if DEBUG_VISION_VERBOSITY > 5
        debug  << "\t\tLineDetection::Post-DecodePenaltySpot" << std::endl;
    #endif
    //qDebug() << "Finnished Decoding Penalty Spots:";

    //
    /*#if DEBUG_VISION_VERBOSITY > 5
    end = clock();
    debug << "\t\t\tLine Detection: " << ((double)end-start)/CLOCKS_PER_SEC * 1000 << " ms" << std::endl;
    debug << "\t\t\tLine Detection: Corner Points: " << ((double)end-startCorner)/CLOCKS_PER_SEC * 1000 << " ms" << std::endl;
    debug << "\t\t\tLine Detection: Field Lines  : " << ((double)startCorner-startLineForm)/CLOCKS_PER_SEC * 1000 << " ms" << std::endl;
    debug << "\t\t\tLine Detection: Line Points  : " << ((double)startLineForm-start)/CLOCKS_PER_SEC * 1000 << " ms" << std::endl;
    #endif
    */
}

void LineDetection::FormLines(FieldObjects* AllObjects,
                              Vision* vision,
                              NUSensorsData* data,
                              std::vector< ObjectCandidate >& candidates,
                              std::vector< TransitionSegment>& leftoverPoints) {


    //Setting up the variables:
    int spacing = vision->getScanSpacings();
    LINE_SEARCH_GRID_SIZE = spacing/4; //Should be 4 at 320width

    int image_height = vision->getImageHeight();
    int image_width = vision->getImageWidth();
    sensorsData = data;

    std::vector<float> ctgvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    if(isOK == false)
    {
        #if DEBUG_VISION_VERBOSITY > 5
            debug << "\t\tNo Camera To Ground Transform, Not running line detection.\n";
        #endif
        #if TARGET_OS_IS_WINDOWS
            qDebug() << "No Camera To Ground Transform, Not running line detection.";
        #endif
        return;
    }
    //clock_t startLineForm = clock();
    //qDebug() << "Finding Lines with segments:  " << linePoints.size();


    /*SHANNON'S NEW LINE DETECTION*/

    //get clusters
    std::vector< std::vector<LinePoint*> > clusters;
    std::vector< LinePoint* > leftover;
    //temps
    std::vector< TransitionSegment > tempseg;
    LinePoint* temppoint;
    std::vector<LinePoint*> tempcluster;
    for(unsigned int i=0; i<candidates.size(); i++)
    {
        //For each ObjectCandidate create std::vector of linepoints and add it to clusters
        tempseg = candidates[i].getSegments();
        tempcluster.clear();
        for(unsigned int k=0; k<tempseg.size(); k++)
        {
            //For each segment create a new linepoint and push it to a std::vector
            temppoint = new LinePoint();
            temppoint->x = (double)tempseg[k].getMidPoint().x;
            temppoint->y = (double)tempseg[k].getMidPoint().y;
            /*
            if(GetDistanceToPoint(*temppoint, convertVals, vision)) {
                SAM::convertPoint(*temppoint, convertVals);
            }
            */
            tempcluster.push_back(temppoint);
        }
        clusters.push_back(tempcluster);
    }

    //get leftover points
    for(unsigned int i=0; i<leftoverPoints.size(); i++)
    {
        temppoint = new LinePoint();
        temppoint->x = (double)leftoverPoints[i].getMidPoint().x;
        temppoint->y = (double)leftoverPoints[i].getMidPoint().y;
        /*
        if(GetDistanceToPoint(*temppoint, convertVals, vision)) {
            SAM::convertPoint(*temppoint, convertVals);
        }
        */
        leftover.push_back(temppoint);
    }
    //prof.split("clusters");

    //POINT CONVERSION TESTING
    /*
    ofstream prepoints, postpoints;
    prepoints.open("prepoints.txt");
    postpoints.open("postpoints.txt");
    Vector3<float> convertVals;
    double xtrans, ytrans;

    for(unsigned int i=0; i<clusters.size(); i++) {
        //for each cluster
        postpoints << clusters[i].size() << "\n";
        for(unsigned int k=0; k<clusters[i].size(); k++) {
            //for each point
            prepoints << clusters[i][k]->x << " " << clusters[i][k]->y << "\n";
            GetDistanceToPoint(*clusters[i][k],convertVals,vision);
            //x = dist * cos(bearing) * cos(elevation)
            xtrans = convertVals[0] * cos(convertVals[1]) * cos(convertVals[2]);
            //y = dist * sin(bearing) * cos(elevation)
            ytrans = convertVals[0] * sin(convertVals[1]) * cos(convertVals[2]);
            postpoints << xtrans << " " << ytrans << "\n";
            //clusters[i][k]->x = xtrans;
            //clusters[i][k]->y = ytrans;
        }
        prepoints << "\n";
        postpoints << "\n";
    }
    //leftovers
    postpoints << leftover.size() << "\n";
    for(unsigned int k=0; k<leftover.size(); k++) {
        //for each point
        prepoints << leftover[k]->x << " " << leftover[k]->y << "\n";
        GetDistanceToPoint(*leftover[k],convertVals,vision);
        //x = dist * cos(bearing) * cos(elevation)
        xtrans = convertVals[0] * cos(convertVals[1]) * cos(convertVals[2]);
        //y = dist * sin(bearing) * cos(elevation)
        ytrans = convertVals[0] * sin(convertVals[1]) * cos(convertVals[2]);
        postpoints << xtrans << " " << ytrans << "\n";
        //leftover[k]->x = xtrans;
        //leftover[k]->y = ytrans;
    }
    */

    //qDebug() << "Beginning SAM:\n";
    std::vector<LSFittedLine*> lines;
    /*OUTPUT FOR DEBUGGING*/
    /*
    ofstream fout;
    fout.open("points.txt");
    for(unsigned int i=0; i<clusters.size(); i++) {
        fout << clusters[i].size() << "\n";
        for(unsigned int k=0; k<clusters[i].size(); k++) {
            fout << clusters[i][k]->x << " " << clusters[i][k]->y << "\n";
        }
    }
    fout << leftover.size() << "\n";
    for(unsigned int i=0; i<leftover.size(); i++) {
        fout << leftover[i]->x << " " << leftover[i]->y << "\n";
    }
    fout.close();
    */
    /*OUTPUT FOR DEBUGGING*/
    //Profiler prof("SHANNON");
    //prof.start();

    SAM::initRules(2.0,2,4,5,8.0,0.999);
    SAM::splitAndMergeLSClusters(lines, clusters, leftover, vision, this, true, true, false);

    //prof.split("SAM");

    //debug << prof;
    /*OUTPUT FOR DEBUGGING*/
    /*
    qDebug() << "printing to file";
    ofstream pointsout;
    ofstream linesout;
    pointsout.open("convertedpoints.txt");
    linesout.open("convertedlines.txt");
    LSFittedLine* line;
    std::vector<LinePoint*> points;
    for(unsigned int i=0; i<lines.size(); i++) {
        line = lines[i];
        //output line
        linesout << line->leftPoint.x << " " << line->findYFromX(line->leftPoint.x) << " " << line->rightPoint.x << " " << line->findYFromX(line->rightPoint.x) << " " << i << " " << line->numPoints << "\n";
        //output line's points
        points = line->getPoints();
        for(unsigned int k=0; k<points.size(); k++) {
            points[k]->inUse = true;
            pointsout << points[k]->x << " " << points[k]->y << " " << i << "\n";
        }
    }
    for(unsigned int i=0; i<clusters.size(); i++) {
        for(unsigned int k=0; k<clusters[i].size(); k++) {
            if(!clusters[i][k]->inUse) {
                pointsout << clusters[i][k]->x << " " << clusters[i][k]->y << " " << -1 << "\n";
            }
        }
    }
    pointsout.close();
    linesout.close();
    qDebug() << "done";
    */
    /*OUTPUT FOR DEBUGGING*/

    //qDebug() << "Finished \nPushing back lines:" << lines.size();
    unsigned int lineno = 0;
    double sumMSD = 0;
    double sumR2 = 0;
    while(!lines.empty()) {
        fieldLines.push_back(*lines.back());
        lineno++;
        //qDebug() << "line " << lineno << " MSD: " << fieldLines.back().getMSD() << " R^2: " << fieldLines.back().getr2tls();
        sumMSD += fieldLines.back().getMSD();
        sumR2 += fieldLines.back().getr2tls();
        lines.pop_back();
    }

    TotalValidLines = lineno;

    //qDebug() << "Average: MSD: " << sumMSD/fieldLines.size() << " R^2: " << sumR2/fieldLines.size();
    //qDebug() << "Finished \n";
    /*SHANNON'S NEW LINE DETECTION*/


    #if DEBUG_VISION_VERBOSITY > 5
        debug << "\t\tTransforming Lines.\n";
    #endif
    TransformLinesToWorldModelSpace(vision);
    #if DEBUG_VISION_VERBOSITY > 5
        debug << "\t\tTransforming Lines " << transformedFieldLines.size() <<".\n";
    #endif

    //qDebug() << "Finding Penalty Spots:";
    #if DEBUG_VISION_VERBOSITY > 5
        debug << "\t\tStart Finding Penalty Spots: \n";
    #endif
    FindPenaltySpot(candidates,vision);
    //qDebug() << "Finding Corner Points:";
    FindCornerPoints(image_width,image_height,vision);

    //qDebug() << "Merging Close Corners: "
    MergeCloseCorners();
    //qDebug() << "Decode Corners:";

    DecodeCorners(AllObjects, vision->m_timestamp, vision);

   // qDebug() << "Decode Penalty Spots:";
    DecodePenaltySpot(AllObjects, vision->m_timestamp);
    //qDebug() << "Finnished Decoding Penalty Spots:";


}


void LineDetection::FindLineOrRobotPoints(ClassifiedSection* scanArea,Vision* vision)
{
    //int image_height = vision->getImageHeight();
    int image_width = vision->getImageWidth();

    int spacing = vision->getScanSpacings();
    LINE_SEARCH_GRID_SIZE = spacing/4; //Should be 4 at 320width

    int numberOfLines = scanArea->getNumberOfScanLines();
    int maxLengthOfScanLine = 0;
    bool robotSegmentIsUsed;


    //! Find Length of Longest ScanLines as we want to only have longest scan lines.
    for(int i = 0; i < numberOfLines; i++)
    {
        if(scanArea->getScanLine(i)->getLength() > maxLengthOfScanLine)
        {
                maxLengthOfScanLine = scanArea->getScanLine(i)->getLength();
        }
    }
    //! Find the LinePoints:
    TransitionSegment* previouslyCloselyScanedSegment = NULL;
    for(int i = 0; i< numberOfLines; i++)
    {

        int numberOfSegments = scanArea->getScanLine(i)->getNumberOfSegments();
        for(int j = 0; j < numberOfSegments ; j++)
        {
            robotSegmentIsUsed = false;
            if(linePoints.size() > MAX_LINEPOINTS) break;
            //if(scanArea->getScanLine(i)->getLength() < maxLengthOfScanLine/1.5) continue;
            TransitionSegment* segment = scanArea->getScanLine(i)->getSegment(j);
            if(segment->getColour() == ClassIndex::pink || segment->getColour() == ClassIndex::shadow_blue || segment->getColour() == ClassIndex::pink_orange || segment->getColour() == ClassIndex::blue)
            {
                robotSegmentIsUsed = true;
                robotSegments.push_back(*segment);
            }
            if(segment->getColour() != ClassIndex::white) continue;
            bool segmentisused = false;
            if(previouslyCloselyScanedSegment == NULL)
            {
                //qDebug() << "Assigning First Previous SEgment";
                previouslyCloselyScanedSegment = segment;
            }
            //int segmentSize = segment->getSize();
            //! CHECK The Length of the segment
            if(segment->getSize() < MIN_POINT_THICKNESS) continue;


            //! LOOKING FOR HORIZONTAL LINE POINTS:
            if(segment->getSize() > VERT_POINT_THICKNESS*0.5 &&
               fabs(previouslyCloselyScanedSegment->getStartPoint().x - segment->getStartPoint().x) >=LINE_SEARCH_GRID_SIZE*2)
            {
                //! CHECK the LEFT AND RIGHT Pixels
                int MidX = (int) (segment->getStartPoint().x + segment->getEndPoint().x)/2;
                int MidY = (int) (segment->getEndPoint().y+segment->getStartPoint().y)/2;
                int LEFTX = 0;
                int RIGHTX = 0;
                if (MidX + VERT_POINT_THICKNESS *1.2>= image_width)
                {
                    RIGHTX  = image_width-1;
                }
                else
                {
                    RIGHTX  = MidX + VERT_POINT_THICKNESS *1.2;
                }
                if (MidX - VERT_POINT_THICKNESS *1.2< 0)
                {
                    LEFTX = 0;
                }
                else
                {
                    LEFTX = MidX - VERT_POINT_THICKNESS *1.2;
                }
                unsigned char LeftColour = vision->classifyPixel(LEFTX, MidY);
                unsigned char RightColour = vision->classifyPixel(RIGHTX, MidY);
                if(LeftColour == ClassIndex::white || RightColour  == ClassIndex::white)
                {
                    robotSegmentIsUsed = true;
                    robotSegments.push_back(*segment);
                }
                if( (LeftColour == ClassIndex::green && RightColour != ClassIndex::white)
                   || (LeftColour != ClassIndex::white && RightColour == ClassIndex::green)
                   //&& segment->getAfterColour() == ClassIndex::green
                   //&& segment->getBeforeColour() == ClassIndex::green
                   )
                {
                    ScanLine tempScanLine;
                    previouslyCloselyScanedSegment = segment;
                    std::vector<unsigned char> colourlist;
                    colourlist.reserve(1);
                    colourlist.push_back(ClassIndex::white);
                    int bufferSize = 10;
                    vision->CloselyClassifyScanline(&tempScanLine,segment,8,ScanLine::DOWN,colourlist,bufferSize);
                    ////qDebug()    << "After Closly Scan: "<<tempScanLine.getNumberOfSegments()
                    //            << segment->getStartPoint().x << "," << segment->getStartPoint().y
                    //            ;


                    for (int k = 0; k < tempScanLine.getNumberOfSegments(); k++)
                    {
                        TransitionSegment* tempSeg = tempScanLine.getSegment(k);
                        ////qDebug()<< k << ": \t" << tempSeg ->getBeforeColour() << "," << tempSeg ->getColour() << ","<<tempSeg ->getAfterColour()
                        //        << "\t" << tempSeg->getSize() << tempSeg->getStartPoint().x << "," << tempSeg->getStartPoint().y;
                        if(tempSeg->getSize() > HORZ_POINT_THICKNESS) continue;
                        //! Check Colour Conditions of segment
                        if (  /*  ((ClassIndex::green   ==  tempSeg->getBeforeColour()) &&
                                (ClassIndex::white   ==  tempSeg->getColour()) &&
                                (ClassIndex::green   ==  tempSeg->getAfterColour())) ||
                                ((ClassIndex::white   ==  tempSeg->getColour())
                            &&  (tempSeg->getAfterColour() == ClassIndex::green && (tempSeg->getBeforeColour() == ClassIndex::unclassified || tempSeg->getBeforeColour() == ClassIndex::shadow_object) )
                            &&  (tempSeg->getBeforeColour() == ClassIndex::green &&(tempSeg->getAfterColour() == ClassIndex::unclassified || tempSeg->getAfterColour() == ClassIndex::shadow_object ) ))*/
                                (ClassIndex::white   ==  tempSeg->getColour())
                                &&  ((tempSeg->getAfterColour() == ClassIndex::green || tempSeg->getAfterColour() == ClassIndex::unclassified) && (tempSeg->getBeforeColour() == ClassIndex::green || tempSeg->getBeforeColour() == ClassIndex::shadow_object || tempSeg->getBeforeColour() == ClassIndex::unclassified))
                                &&  ((tempSeg->getBeforeColour() == ClassIndex::green || tempSeg->getBeforeColour() == ClassIndex::unclassified) && (tempSeg->getAfterColour() == ClassIndex::green ||  tempSeg->getAfterColour() == ClassIndex::shadow_object  || tempSeg->getAfterColour() == ClassIndex::unclassified)))


                        {
                                //qDebug() << "Attempting to add point";
                                Vector2<int> linepointposition= tempSeg->getMidPoint();
                                //ADD A FIELD LINEPOINT!



                                LinePoint tempLinePoint;
                                tempLinePoint.width = tempSeg->getSize();
                                tempLinePoint.x = linepointposition.x;
                                tempLinePoint.y = linepointposition.y;
                                tempLinePoint.inUse = false;
                                //CUT CLOSE LINE POINTS OFF
                                bool canNotAdd = false;
                                for (unsigned int num =0; num <linePoints.size(); num++)
                                {

                                    if((fabs(tempLinePoint.x - linePoints[num].x) <= LINE_SEARCH_GRID_SIZE*1.5) &&
                                       (fabs(tempLinePoint.y - linePoints[num].y) <= LINE_SEARCH_GRID_SIZE*1.5))
                                    {
                                        canNotAdd = true;

                                        break;
                                    }
                                }
                                if(!canNotAdd)
                                {
                                    segmentisused = true;
                                    tempLinePoint.inUse = false;
                                    linePoints.push_back(tempLinePoint);
                                    verticalLineSegments.push_back(*tempSeg);
                                    //qDebug() << "Added LinePoint to list: "<< tempLinePoint.x <<"," <<tempLinePoint.y << tempLinePoint.width;
                                }
                            }
                    }
                }
            }

            //CHECK COLOUR(GREEN-WHITE-GREEN Transistion)
            //CHECK COLOUR (U-W-G or G-W-U Transistion)
            if(    ((ClassIndex::white   ==  segment->getColour()) &&

                     ((segment->getAfterColour() == ClassIndex::green || segment->getAfterColour() == ClassIndex::unclassified)  && (segment->getBeforeColour() == ClassIndex::green
                         || segment->getBeforeColour() == ClassIndex::shadow_object || segment->getBeforeColour() == ClassIndex::unclassified ) ))

                     && ((segment->getBeforeColour() == ClassIndex::green || segment->getBeforeColour() == ClassIndex::unclassified)  && (segment->getAfterColour() == ClassIndex::green
                         || segment->getAfterColour() == ClassIndex::shadow_object || segment->getAfterColour() == ClassIndex::unclassified )) )

            {
                //ADD A FIELD LINEPOINT!
                Vector2<int>linepointposition = segment->getMidPoint();


                LinePoint tempLinePoint;
                tempLinePoint.width = (int)segment->getSize();
                tempLinePoint.x = linepointposition.x;
                tempLinePoint.y = linepointposition.y;
                tempLinePoint.inUse = false;
                //CUT CLOSE LINE POINTS OFF
                bool canNotAdd = false;
                for (unsigned int num =0; num <linePoints.size(); num++)
                {

                    if((fabs(tempLinePoint.x - linePoints[num].x) <= LINE_SEARCH_GRID_SIZE) &&
                       (fabs(tempLinePoint.y - linePoints[num].y) <= LINE_SEARCH_GRID_SIZE))
                    {
                        canNotAdd = true;
                        break;
                    }
                }
                if(!canNotAdd)
                {
                    segmentisused = true;
                    tempLinePoint.inUse = false;
                    linePoints.push_back(tempLinePoint);
                    horizontalLineSegments.push_back(*segment);
                }
                 ////qDebug() << "Found LinePoint (MidPoint): "<< (start.x + end.x) / 2 << ","<< (start.y+end.y)/2 << " Length: "<< segment->getSize();
                //LinePointCounter++;

            }

            if(segmentisused == false && robotSegmentIsUsed == false)
            {
                robotSegments.push_back(*segment);
            }


        }

    }
    return;
}

void LineDetection::FindFieldLines(int IMAGE_WIDTH, int IMAGE_HEIGHT){

    //qDebug("Start Find Lines..\n");

    //Try and make lines now...
    //int DistanceStep;
    int SearchMultiplier = 1.5; // Increases GRID SEARCH SIZE via a multiple of SearchMultiplier
    double ColSlopeVal;
    int previousPointID;

    int MAX_SCAN_SPACING = LINE_SEARCH_GRID_SIZE * 4; //ORIGINAL SCANLINE SPACINGS were 4x the
    int GRID = MAX_SCAN_SPACING * SearchMultiplier;
    //Lines 'should' have a point on the same search pattern that maches them, which should also be close to them..
    //First look for horz lines (from the vert search grid..)
    // ***********************************************************************************************************************
    if ( linePoints.size() < MIN_POINTS_ON_LINE_FINAL)
    {
        return;
    }
    //SORT THE LINES BY Y then BY X:
    //clock_t startSort = clock();
    //qsort(linePoints,0,linePoints.size()-1,2);

    //HORIZONTAL Line Search:
    //clock_t startHorizontalSearch = clock();
    //debug << "Line Detection: Field Lines  : Sorting: " << (double)(startHorizontalSearch - startSort )/ CLOCKS_PER_SEC * 1000 << " ms"<<std::endl;
    //Only bother searching if there is enough points to make part of a line..
    for (unsigned int SearchFrom = 0; SearchFrom < linePoints.size() ; SearchFrom++)
    {   //for all line points recorded
        if(fieldLines.size()> MAX_FIELDLINES) break;
        if(linePoints[SearchFrom].inUse) continue;
        //if(linePoints[SearchFrom].width > VERT_POINT_THICKNESS) continue;  //STOP if LINE is too THICK, but can use if in Vertical Line Search.
        for (unsigned int EndCheck = SearchFrom+1; EndCheck < linePoints.size()-1; EndCheck++){ 	//for remaining points recorded
            if (linePoints[EndCheck].width > VERT_POINT_THICKNESS) continue; //STOP if LINE is too THICK, but can use if in Vertical Line Search.
            if ((linePoints[EndCheck].inUse == true)) continue;
            if ((linePoints[EndCheck].x == linePoints[SearchFrom].x))continue; //Vertical Line
            // Skip all points on the same search line as this one or have already been removed..

            if (linePoints[EndCheck].x <= linePoints[SearchFrom].x + GRID)
            {
                int DistanceStep = fabs(linePoints[EndCheck].x-linePoints[SearchFrom].x)/(MAX_SCAN_SPACING);  //number of grid units long

                if (linePoints[EndCheck].y <= linePoints[SearchFrom].y + GRID * DistanceStep)
                {
                    //qDebug() << linePoints[SearchFrom].x<< linePoints[SearchFrom].y << linePoints[EndCheck].x <<linePoints[EndCheck].y << GRID <<  fabs(linePoints[SearchFrom].y - linePoints[EndCheck].y);

                    //We've found what might be a line, so lets see if we can find any more lines that match this one..
                    LSFittedLine tempFieldLine;
                    tempFieldLine.addPoint(linePoints[SearchFrom]);
                    tempFieldLine.addPoint(linePoints[EndCheck]);
                    previousPointID = EndCheck;
                    ColSlopeVal = linePoints[SearchFrom].y - linePoints[EndCheck].y;
                    //loop through the rest of the points that maybe in this 'line'
                    for (unsigned int PointID = EndCheck+1; PointID < linePoints.size(); PointID++)
                    {
                        if(linePoints[PointID].x >  linePoints[previousPointID].x + GRID )
                        {
                            break;
                        }
                        if (linePoints[previousPointID].x == linePoints[PointID].x) continue; //Vertical Line
                        if (linePoints[PointID].inUse == true) continue;

                        if (    linePoints[PointID].x <= linePoints[previousPointID].x + GRID
                            &&  linePoints[PointID].y <= linePoints[previousPointID].y + GRID )
                        {
                            double DisMod = (linePoints[PointID].x - linePoints[previousPointID].x)/(double)(MAX_SCAN_SPACING);
                            //qDebug() << "\tCheck Point : " << fabs(linePoints[PointID].y+(ColSlopeVal*DisMod) - linePoints[previousPointID].y) << DisMod << linePoints[previousPointID].x<< linePoints[previousPointID].y << linePoints[PointID].x <<linePoints[PointID].y;
                            //Check if the slope is about right..
                            if (fabs(linePoints[PointID].y+(ColSlopeVal*DisMod) - linePoints[previousPointID].y) <= 1)
                            {
                                //This is another point on the line..
                                //qDebug() << "\t   Point Added: " <<linePoints[previousPointID].x<< linePoints[previousPointID].y << linePoints[PointID].x <<linePoints[PointID].y;
                                tempFieldLine.addPoint(linePoints[PointID]);
                                previousPointID = PointID;
                            }
                        }
                    }
                    if(tempFieldLine.numPoints > MIN_POINTS_ON_LINE-1)
                    {
                        fieldLines.push_back(tempFieldLine);
                    }
                    else
                    {
                        tempFieldLine.clearPoints();
                    }
                }
            }
            else
            {
                EndCheck = linePoints.size();
            }

        }
    }

    //Now do all that again, but this time looking for the vert lines from the horz search grid..
    //unsigned int horizontalEnd = fieldLines.size();
    //SORT POINTS
    //qDebug() << "SORTING...";
    //clock_t startSortAgain = clock();
    //debug << "Line Detection: Field Lines  : Horizontal Search: " << (double)(startSortAgain - startHorizontalSearch )/ CLOCKS_PER_SEC * 1000 << " ms"<<std::endl;
    qsort(linePoints,0,linePoints.size()-1,1);
    /*for (size_t i = 0; i < linePoints.size() ; i++)
    {
        qDebug() << i << ": "<< linePoints[i].x << "," << linePoints[i].y;

    }*/
    //qDebug() << "Finnished...";

    //clock_t startVerticalSearch = clock();
    //debug << "Line Detection: Field Lines  : Sort: " << (double)(startVerticalSearch -  startSortAgain )/ CLOCKS_PER_SEC * 1000 << " ms"<<std::endl;
    for (unsigned int SearchFrom = 0; SearchFrom < linePoints.size() ; SearchFrom++){
        if(fieldLines.size()> MAX_FIELDLINES) break;
        if(linePoints[SearchFrom].inUse) continue;

        for (unsigned int EndCheck = SearchFrom+1; EndCheck < linePoints.size(); EndCheck++){
                //std::cout << "Comparing.."<< SearchFrom << " with " << EndCheck <<std::endl;
                //Skip all points on the same search line as this one or have already been removed..

            if (linePoints[EndCheck].inUse == true) continue;
            if (linePoints[EndCheck].y == linePoints[SearchFrom].y) continue; //Horizontal Line
            //if (linePoints[EndCheck].width > HORZ_POINT_THICKNESS) continue;  //STOP if LINE is too THICK, but can use if in Vertical Line Search.
            //if (linePoints[EndCheck].width < MIN_POINT_THICKNESS*3) continue;
            if (linePoints[EndCheck].y <= linePoints[SearchFrom].y + GRID)
            {
                int DistanceStep = fabs(linePoints[EndCheck].y-linePoints[SearchFrom].y)/MAX_SCAN_SPACING;
                //qDebug() << linePoints[SearchFrom].x<< linePoints[SearchFrom].y << linePoints[EndCheck].x <<linePoints[EndCheck].y << LINE_SEARCH_GRID_SIZE << DistanceStep<< fabs(linePoints[SearchFrom].y - linePoints[EndCheck].y);

                if (linePoints[EndCheck].x <= linePoints[SearchFrom].x + GRID * DistanceStep)
                {
                //We've found what might be a line, so lets see if we can find any more lines that match this one..
                    //std::cout << "Starting New vertical Line" << std::endl;
                    LSFittedLine tempFieldLine;
                    tempFieldLine.addPoint(linePoints[SearchFrom]);
                    tempFieldLine.addPoint(linePoints[EndCheck]);
                    previousPointID = EndCheck;
                    ColSlopeVal = linePoints[SearchFrom].x - linePoints[EndCheck].x;

                    for (unsigned int PointID = EndCheck+1; PointID < linePoints.size(); PointID++)
                    {
                        if(linePoints[PointID].y > linePoints[previousPointID].y + GRID)
                        {
                                //We've moved too far to keep searching..
                                break;
                        }
                        if (linePoints[PointID].width > HORZ_POINT_THICKNESS) continue;
                        if (linePoints[PointID].inUse == true) continue;
                        if (linePoints[previousPointID].y == linePoints[PointID].y) continue; //Horizontal Line

                        if (    linePoints[PointID].y <= linePoints[previousPointID].y + GRID
                             && linePoints[PointID].x <= linePoints[previousPointID].x + GRID )
                        {
                            double DisMod = (linePoints[PointID].y - linePoints[previousPointID].y)/MAX_SCAN_SPACING;
                            //Check if the slope is about right..
                            if (fabs(linePoints[PointID].x+(ColSlopeVal*DisMod) - linePoints[previousPointID].x) <= 1){
                                //This is another point on the line..
                                tempFieldLine.addPoint(linePoints[PointID]);
                                previousPointID = PointID;
                            }
                        }

                    }
                    if(tempFieldLine.numPoints > MIN_POINTS_ON_LINE-1)
                    {
                        fieldLines.push_back(tempFieldLine);
                    }
                    else
                    {
                        tempFieldLine.clearPoints();
                    }
                }
            }
            else {
                    EndCheck = linePoints.size();
            }
        }
    }

    //clock_t startJoining = clock();
    //debug << "Line Detection: Field Lines  : Vertical Search: " << (double)(startJoining - startVerticalSearch )/ CLOCKS_PER_SEC * 1000 << " ms"<<std::endl;

    //---------------------------------------------------
    // START OF JOINING LINES
    //---------------------------------------------------

    unsigned int LineIDStart;

    //Drop out here if there isn't any point continuing.. DEBUG:
    if (fieldLines.size() < 1)
    {
        //printf("No Lines... ABORT!!!!");
            return;
    }
    else{
        //printf("END FieldLinesCounter: %i\n", fieldLines.size());
    }

    //for(int l = 0; l < FieldLinesCounter; l++)
    //{
    //	std::vector<LinePoint*> points = fieldLines[l].getPoints();
    //	std::cout << "Line " << l << " ";
    //	for (unsigned int p = 0; p < points.size(); p++)
    //	{
    //		std::cout << "("<< points[p]->x <<"," <<points[p]->y << ") ";
    //	}
    //	std::cout << std::endl;
    //}
    /*for (unsigned int i = 0; i < fieldLines.size(); i++)
    {
        qDebug() << i<< ": \t Valid: "<<fieldLines[i].valid
                << " \t Start(x,y): ("<< fieldLines[i].leftPoint.x<<","<< fieldLines[i].leftPoint.y
                << ") \t EndPoint(x,y):(" << fieldLines[i].rightPoint.x<<","<< fieldLines[i].rightPoint.y<< ")"
                << "\t Number of LinePoints: "<< fieldLines[i].numPoints;
        std::vector<LinePoint*> points = fieldLines[i].getPoints();

        for(unsigned int j = 0; j < points.size() ; j ++)
        {
            qDebug() << "{" << points[j]->x << ","<<  points[j]->y << "} ";
        }

    }*/

    //We should now have about 10 lines max, some of which can be joined together (since they may be two lines seperated by a break but otherwise one line really...)
    for (LineIDStart = 0; LineIDStart < fieldLines.size()-1; LineIDStart++){

        if (!fieldLines[LineIDStart].valid || fieldLines[LineIDStart].numPoints < MIN_POINTS_ON_LINE) continue;   	// this apears to me to be first use of 'validLine' so how does it get to be true? ALEX

        for (unsigned int LineIDEnd = LineIDStart+1; LineIDEnd < fieldLines.size(); LineIDEnd++)
        {
            //if (LineIDStart < horizontalEnd && LineIDEnd > horizontalEnd) break;

            if (!fieldLines[LineIDEnd].valid || fieldLines[LineIDStart].numPoints < MIN_POINTS_ON_LINE) continue;
            //Try extending the lines so they are near the ends of the other ones, and see if their in any way close...
            //qDebug() << "Trying to join: " << LineIDStart << " \t with " << LineIDEnd;
            //We need to check both, since in the actual points can be close enough together to make a small short segment
            //of a corner be added to a line, while the far comparision points allow lines which are fairly para get joined
            //togther.. by checking both, only the correct lines are actually joined...

            double MSD1, MSD2, r2tls1, r2tls2;
            double sxx, syy, sxy, Sigma;
            LSFittedLine Line1 = fieldLines[LineIDStart];
            LSFittedLine Line2 = fieldLines[LineIDEnd];
            double L1sumCompX, L1sumCompY, L1sumCompXY, L1sumCompX2, L1sumCompY2;
            double L2sumCompX, L2sumCompY, L2sumCompXY, L2sumCompX2, L2sumCompY2;
            //Working Out Variables for comparision:

            //if(Line1.getGradient() > 1){
                L1sumCompX =    IMAGE_WIDTH;
                L1sumCompY =    Line1.getGradient()*IMAGE_WIDTH + 2*Line1.getYIntercept();
                L1sumCompX2 =   L1sumCompX*L1sumCompX;
                L1sumCompY2 =   (Line1.getYIntercept())*(Line1.getYIntercept())+
                                ((Line1.getGradient()*IMAGE_WIDTH + Line1.getYIntercept())*
                                (Line1.getGradient()*IMAGE_WIDTH + Line1.getYIntercept()));
                L1sumCompXY =   L1sumCompX * (Line1.getGradient()*IMAGE_WIDTH + Line1.getYIntercept());

                L2sumCompX =    IMAGE_WIDTH;
                L2sumCompY =    Line2.getGradient()*IMAGE_WIDTH + 2*Line2.getYIntercept();
                L2sumCompX2 =   L2sumCompX*L2sumCompX;
                L2sumCompY2 =   (Line2.getYIntercept())*(Line2.getYIntercept())+
                                ((Line2.getGradient()*IMAGE_WIDTH + Line2.getYIntercept())*
                                (Line2.getGradient()*IMAGE_WIDTH + Line2.getYIntercept()));
                L2sumCompXY = L2sumCompX * (Line2.getGradient()*IMAGE_WIDTH + Line2.getYIntercept());
            /*}
            else{ //Switch the X and Ys around
                L1sumCompX =    Line1.getGradient()*IMAGE_HEIGHT + 2*Line1.getXIntercept(); //IMAGE_WIDTH;
                L1sumCompY =    IMAGE_HEIGHT;//Line1.getGradient()*IMAGE_WIDTH + 2*Line1.getYIntercept();
                L1sumCompX2 =   (Line1.getXIntercept())*(Line1.getXIntercept())+
                                ((Line1.getGradient()*IMAGE_HEIGHT + Line1.getXIntercept())*
                                (Line1.getGradient()*IMAGE_HEIGHT + Line1.getXIntercept()));
                L1sumCompY2 =   L1sumCompY*L1sumCompY;//(Line1.getYIntercept())*(Line1.getYIntercept())+
                                //((Line1.getGradient()*IMAGE_WIDTH + Line1.getYIntercept())*
                                //(Line1.getGradient()*IMAGE_WIDTH + Line1.getYIntercept()));
                L1sumCompXY =   L1sumCompY * (Line1.getGradient()*IMAGE_HEIGHT + Line1.getXIntercept());

                L2sumCompX =    Line2.getGradient()*IMAGE_HEIGHT + 2*Line2.getXIntercept();//IMAGE_WIDTH;
                L2sumCompY =    IMAGE_HEIGHT;//Line2.getGradient()*IMAGE_WIDTH + 2*Line2.getYIntercept();
                L2sumCompX2 =   (Line2.getXIntercept())*(Line2.getXIntercept())+
                                ((Line2.getGradient()*IMAGE_HEIGHT + Line2.getXIntercept())*
                                (Line2.getGradient()*IMAGE_HEIGHT + Line2.getXIntercept()));//L2sumCompX*L2sumCompX;
                L2sumCompY2 =   L2sumCompY*L2sumCompY;//(Line2.getYIntercept())*(Line2.getYIntercept())+
                                //((Line2.getGradient()*IMAGE_WIDTH + Line2.getYIntercept())*
                                //(Line2.getGradient()*IMAGE_WIDTH + Line2.getYIntercept()));
                L2sumCompXY = L2sumCompY * (Line2.getGradient()*IMAGE_HEIGHT+ Line2.getXIntercept());
            }*/

            sxx = L2sumCompX2+L1sumCompX2 - (L2sumCompX+L1sumCompX)*(L2sumCompX+L1sumCompX)/4;
            syy = L2sumCompY2+L1sumCompY2 - (L2sumCompY+L1sumCompY)*(L2sumCompY+L1sumCompY)/4;
            sxy = L2sumCompXY+L1sumCompXY - (L2sumCompX+L1sumCompX)*(L2sumCompY+L1sumCompY)/4;

            Sigma = (sxx+syy-sqrt((sxx-syy)*(sxx-syy)+4*sxy*sxy))/2;
            MSD1 = Sigma/4;
            r2tls1 = 1.0-(4.0*Sigma*Sigma/((sxx+syy)*(sxx+syy)+(sxx-syy)*(sxx-syy)+4.0*sxy*sxy));


            //JOIN Copies of LINEs:
            Line1.joinLine(Line2);
            MSD2 = Line1.getMSD();
            r2tls2 = Line1.getr2tls();


            //Now make sure the slopes are both about the same degree angle....
            // Seems to have a problem with lines "within" other lines, so pick them out..
            //qDebug() << "Joining Line " <<LineIDStart <<"-"<<LineIDEnd <<": " <<r2tls1 << "," <<r2tls2 << ", "<<MSD1 << ", "<<MSD2;
            if ((r2tls1 > .80 && r2tls2 > .80 && MSD1 < 50 && MSD2 < 50))// || (r2tls1 > .90 && r2tls2 > .90 && MSD2 < 20 && fabs(Line1.getGradient()) > 1))                    // (.90 & 40)alex CAN ADJUST THIS FOR LINE JOINING
            {
                //They are the same line, so join them together...
                //qDebug() << "Joining Lines: "<< LineIDEnd<< " to "<<LineIDStart;
                fieldLines[LineIDStart].joinLine(fieldLines[LineIDEnd]);
                //std::cout << "Num Points Line2: "<< fieldLines[LineIDEnd].numPoints <<std::endl;
                fieldLines[LineIDEnd].clearPoints();
                //! Reset ans start again:
                //LineIDStart = 0;
                //LineIDEnd = LineIDStart+1;

            }
        }
    }

    //Remove any lines that are still too small (or really badly fitted)..
    for (LineIDStart = 0; LineIDStart < fieldLines.size(); LineIDStart++){
        if (!fieldLines[LineIDStart].valid) continue;
        if (fieldLines[LineIDStart].numPoints < MIN_POINTS_ON_LINE_FINAL  || fieldLines[LineIDStart].getr2tls() < .85)
        {// alex ADJUST CAN HELP TO DELETE CIRCLE LINES
            fieldLines[LineIDStart].valid = false;

            //std::cout<<"Line " << LineIDStart << " was Removed." << std::endl;
        }
        else {
            TotalValidLines++;
            //qDebug( "VaildLine Found: %i @ %i\n" ,TotalValidLines, LineIDStart);
        }

    }



    /*for (unsigned int i = 0; i < fieldLines.size(); i++)
    {
        qDebug() << i<< ": \t Valid: "<<fieldLines[i].valid
                << " \t Start(x,y): ("<< fieldLines[i].leftPoint.x<<","<< fieldLines[i].leftPoint.y
                << ") \t EndPoint(x,y):(" << fieldLines[i].rightPoint.x<<","<< fieldLines[i].rightPoint.y<< ")"
                << "\t Number of LinePoints: "<< fieldLines[i].numPoints;
        std::vector<LinePoint*> points = fieldLines[i].getPoints();

        for(unsigned int j = 0; j < points.size() ; j ++)
        {
            qDebug() << "{" << points[j]->x << ","<<  points[j]->y << "} ";
        }

    }*/

    //clock_t end = clock();
    //debug << "Line Detection: Field Lines  : Joining Search: " << (double)(end - startJoining )/ CLOCKS_PER_SEC * 1000 << " ms"<<std::endl;

}

/**-------------
//Penalty spot: a small line in the middle of the field with no surrounding lines
        //1. For each line less then 1/4 of image:
                //2. find scan points about line
                //3. perform scan for whites on classified image
                        //3a. 3x3 pixel search about point
                //4. If no white found then we have penalty spot
-------------**/

void LineDetection::FindPenaltySpot(std::vector< ObjectCandidate >& candidates, Vision* vision)
{
        //int IMAGE_WIDTH = vision->getImageWidth();

    int lx,ly,rx,ry, mx, my;
    double lineLength;
    lx = 0;
    ly = 0;
    rx = 0;
    ry = 0;
    mx = 0;
    my = 0;
    lineLength = 0.0;
    //qDebug() << "Number of Lines to check for Pentaly Spot : " <<fieldLines.size();
    #if DEBUG_VISION_VERBOSITY > 5
        debug << "\t\t\tStart Finding Penalty Spots: "<< fieldLines.size() <<" Lines.\n";
    #endif
    //CHECK LINES:
    /*for (unsigned int i = 0; i < fieldLines.size(); i++)
    {
        //CHECK ALL LINES (EVEN IF NOT A LINE!)
        lx = transformedFieldLines[i].leftPoint.x;
        ly = transformedFieldLines[i].leftPoint.y;
        rx = transformedFieldLines[i].rightPoint.x;
        ry = transformedFieldLines[i].rightPoint.y;
        mx = (int)((lx+rx)/2.0);
        my = (int)((ly+ry)/2.0);
        lineLength = sqrt((lx-rx)*(lx-rx) + (ly-ry)*(ly-ry));

        if (lineLength < 20)
        {
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Line length less then 20.\n";
            #endif
            //CHECK FOR WHITES!
            //Left Point is Lower X, Right Point is Higher X
            //Perform checks on:
            //	Left Point:	(lx-0.5length,ly), (lx, ly+3/4length), (lx, ly-3/4length),
            //	MidPoint:	(mx, my-3/4length), (mx, my-3/4length)
            //	Right Point:	(rx+0.5length,ry), (rx, ry+3/4length), (lx, ry-3/4length),
            lx = fieldLines[i].leftPoint.x;
            ly = fieldLines[i].leftPoint.y;
            rx = fieldLines[i].rightPoint.x;
            ry = fieldLines[i].rightPoint.y;
            mx = (int)((lx+rx)/2.0);
            my = (int)((ly+ry)/2.0);
            lineLength = sqrt((lx-rx)*(lx-rx) + (ly-ry)*(ly-ry));
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<"Look for Whites around the penalty spot.\n";
            #endif
            //qDebug() << "\t\t_______________CHECK PENALTY SPOT FOUND!!!___________ at ("  << mx << ","<<  my << ")"<< std::endl;
            if(checkAroundForWhite(lx,ly,mx, my,rx,ry, lineLength, vision)) continue;

            //qDebug() << "\t\t_______________PENALTY SPOT FOUND!!!___________ at ("  << mx << ","<<  my << ")"<< std::endl;
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<"No Whites around the penalty spot.\n";
            #endif
            Vector2<float> screenPositionAngle(vision->CalculateBearing(mx), vision->CalculateElevation(my));
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Found Screen Angle.\n";
            #endif

            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Calculate Distance to point.\n";
            #endif
            double TempDist = 0;
            double TempBearing = 0;
            double TempElev = 0;
            //Vector3<float> point;
            //bool isOK = GetDistanceToPoint(*midPoint, point, vision);
            bool isOK = GetDistanceToPoint(mx, my, &TempDist, &TempBearing, &TempElev, vision);
            if(!isOK)
            {
                #if DEBUG_VISION_VERBOSITY > 0
                    debug << "\t\t\tLineDetection::FindPenaltySpot: Distance to point Failed.";
                #endif
                return;
            }

            //qDebug() << "Distance:\t"<< TempDist<<  "\tBearing:\t"<< TempBearing <<  "\tElevation:\t" << TempElev<< std::endl;
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Distance to point Success.\n";
            #endif
            int TempID = FieldObjects::FO_PENALTY_UNKNOWN;
            AmbiguousObject tempUnknownPenalty(TempID, "Unknown Penalty");
            tempUnknownPenalty.addPossibleObjectID(FieldObjects::FO_PENALTY_YELLOW);
            tempUnknownPenalty.addPossibleObjectID(FieldObjects::FO_PENALTY_BLUE);
            Vector3<float> measured((float)TempDist,(float)TempBearing,(float)TempElev);
            Vector3<float> measuredError(0.0,0.0,0.0);
            Vector2<int> screenPosition(mx, my);
            Vector2<int> sizeOnScreen(8,2);
            tempUnknownPenalty.UpdateVisualObject(measured,measuredError,screenPositionAngle,screenPosition,sizeOnScreen,vision->m_timestamp);
            possiblePenaltySpots.push_back(tempUnknownPenalty);
        }
        else
        {
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Line too long.\n";
            #endif
            //std::cout << "Line["<< i<< "] too long."<< std::endl;
            continue;
        }
    }
    */
    //Check Candidates:
    for (unsigned int i = 0; i < candidates.size(); i++)
    {
        //Check diagonal distance of candidate:
        lx = candidates[i].getTopLeft().x;
        ly = candidates[i].getTopLeft().y;
        rx = candidates[i].getBottomRight().x;
        ry = candidates[i].getBottomRight().y;

        Vector3<float> leftWMPolarPoint, rightWMPolarPoint;
        //Get Transformed Points:
        Point leftPoint(lx,ly);
        bool isOKA = GetDistanceToPoint(leftPoint, leftWMPolarPoint, vision);
        Point rightPoint(rx,ry);
        bool isOKB = GetDistanceToPoint(rightPoint, rightWMPolarPoint, vision);

        if (!isOKA || !isOKB)
            continue;
        //Get Difference between tranformed points:
        //relative x,y:
        float relLeftX = leftWMPolarPoint.x * sin(leftWMPolarPoint.y);
        float relLeftY = leftWMPolarPoint.x * cos(leftWMPolarPoint.y);
        float relRightX = rightWMPolarPoint.x * sin(rightWMPolarPoint.y);
        float relRightY = rightWMPolarPoint.x * cos(rightWMPolarPoint.y);

        float length = sqrt((relLeftX - relRightX)*(relLeftX - relRightX) + (relLeftY - relRightY)*(relLeftY - relRightY));



        //qDebug() << "Penalty Length "<<length;
        if (length < 30)
        {

            //CHECK FOR WHITES!
            //Left Point is Lower X, Right Point is Higher X
            //Perform checks on:
            //	Left Point:	(lx-0.5length,ly), (lx, ly+3/4length), (lx, ly-3/4length),
            //	MidPoint:	(mx, my-3/4length), (mx, my-3/4length)
            //	Right Point:	(rx+0.5length,ry), (rx, ry+3/4length), (lx, ry-3/4length),

            mx = (int)((lx+rx)/2.0);
            my = (int)((ly+ry)/2.0);
            lx = candidates[i].getTopLeft().x - 16;
            ly = my;
            rx = candidates[i].getBottomRight().x + 16;
            ry = my;

            lineLength = fabs(lx-rx);

            //qDebug() << "\t\t_______________CHECK PENALTY SPOT FOUND!!!___________ at ("  << mx << ","<<  my << ")"<< std::endl;
            if(checkAroundForWhite(lx,ly,mx, my,rx,ry, lineLength, vision)) continue;

            //qDebug() << "\t\t_______________PENALTY SPOT FOUND!!!___________ at ("  << mx << ","<<  my << ")"<< std::endl;
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<"No Whites around the penalty spot.\n";
            #endif
            Vector2<float> screenPositionAngle(vision->CalculateBearing(mx), vision->CalculateElevation(my));
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Found Screen Angle.\n";
            #endif

            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Calculate Distance to point.\n";
            #endif
            double TempDist = 0;
            double TempBearing = 0;
            double TempElev = 0;
            //Vector3<float> point;
            //bool isOK = GetDistanceToPoint(*midPoint, point, vision);
            bool isOK = GetDistanceToPoint(mx, my, &TempDist, &TempBearing, &TempElev, vision);
            if(!isOK)
            {
                #if DEBUG_VISION_VERBOSITY > 0
                    debug << "\t\t\tLineDetection::FindPenaltySpot: Distance to point Failed.";
                #endif
                return;
            }

            //qDebug() << "Distance:\t"<< TempDist<<  "\tBearing:\t"<< TempBearing <<  "\tElevation:\t" << TempElev<< std::endl;
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Distance to point Success.\n";
            #endif
            int TempID = FieldObjects::FO_PENALTY_UNKNOWN;
            AmbiguousObject tempUnknownPenalty(TempID, "Unknown Penalty");
            tempUnknownPenalty.addPossibleObjectID(FieldObjects::FO_PENALTY_YELLOW);
            tempUnknownPenalty.addPossibleObjectID(FieldObjects::FO_PENALTY_BLUE);
            Vector3<float> measured((float)TempDist,(float)TempBearing,(float)TempElev);
            Vector3<float> measuredError(0.0,0.0,0.0);
            Vector2<int> screenPosition(mx, my);
            Vector2<int> sizeOnScreen(8,2);
            tempUnknownPenalty.UpdateVisualObject(measured,measuredError,screenPositionAngle,screenPosition,sizeOnScreen,vision->m_timestamp);
            possiblePenaltySpots.push_back(tempUnknownPenalty);
        }
        else
        {
            #if DEBUG_VISION_VERBOSITY > 5
                debug << "\t\t\tPenalty Spots: "<< i <<" Line too long.\n";
            #endif
            //std::cout << "Line["<< i<< "] too long."<< std::endl;
            continue;
        }
    }


}

bool LineDetection::checkAroundForWhite(int mx, int my,double length, Vision* vision)
{
    bool whitePixelFound;
    int checkX, checkY;
    int searchRadius = (int)(length/16);
    if (searchRadius < 2)
    {
            searchRadius = 2;
    }
    //qDebug() << "\tChecking Left " ;
    //Mid Point Left: (lx-0.5length,ly)
    checkX = mx - (int)(length);
    checkY = my;
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;
    //qDebug() << "\tChecking Right " ;
    checkX = mx + (int)(length);
    checkY = my;
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;
    //qDebug() << "\tChecking Bottom " ;
    //Mid Point1: (mx, my-3/4length)
    checkX = mx ;
    checkY = my - (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;
    //qDebug() << "\tChecking Top " ;
    //Mid Point2: (mx, my+3/4length)
    checkX = mx ;
    checkY = my + (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    return false;
}
bool LineDetection::checkAroundForWhite(int lx, int ly, int mx, int my, int rx, int ry,double length, Vision* vision)
{
    bool whitePixelFound;
    int checkX, checkY;
    int searchRadius = (int)(length/16);
    if (searchRadius < 2)
    {
            searchRadius =2;
    }
    //Left Point 1: (lx-0.5length,ly)
    checkX = lx - (int)(length);
    checkY = ly;
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    //Left Point 2: (lx, ly+3/4length)
    checkX = lx ;
    checkY = ly + (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    //Left Point3: (lx, ly-3/4length)
    checkX = lx ;
    checkY = ly - (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    //Mid Point1: (mx, my-3/4length)
    checkX = mx ;
    checkY = my - (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    //Mid Point2: (mx, my+3/4length)
    checkX = mx ;
    checkY = my + (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    //Right Point1: (rx+0.5length,ry)
    checkX = rx + (int)(length);
    checkY = ry;
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    //Right Point2: (rx, ry+3/4length)
    checkX = rx;
    checkY = ry+ (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    //Right Point2: (rx, ry-3/4length)
    checkX = rx;
    checkY = ry - (int)(3*length/4);
    whitePixelFound = DetectWhitePixels(checkX,checkY, searchRadius, vision);
    if(whitePixelFound)
            return true;

    return false;
}

bool LineDetection::DetectWhitePixels(int checkX, int checkY, int searchRadius,Vision* vision)
{
        //Search -searchRadius  to +searchRadius
        bool whitePixelFound = false;
        for (int i = checkX - searchRadius; i < checkX + searchRadius; i++)
        {
                for (int j =checkY - searchRadius; j< checkY + searchRadius; j++)
                {
                    
                    if(vision->isPixelOnScreen(i,j))
                    {
                        if(vision->classifyPixel(i,j) == ClassIndex::white)
                        {
                                whitePixelFound = true;
                                break;
                        }
                    }
                    else
                    {
                        whitePixelFound = true;
                    }
                }
                if(whitePixelFound)
                        break;
        }
        return whitePixelFound;
}


/*----------------------
// Method:      TransformLinesToWorldModelSpace
// Arguments:   void
// Returns:     void
// Description: Changes cordinates from image space to relative world model space
--------------------------*/

void  LineDetection::TransformLinesToWorldModelSpace(Vision* vision)
{
    #if TARGET_OS_IS_WINDOWS
        qDebug() << "TransformLinesToWorldModelSpace" <<  fieldLines.size();
    #endif
    // Itterate though obtained field lines and calculate the eqivelent line in relative WM Space
    for(unsigned int i = 0; i< fieldLines.size(); i++)
    {
        //std::vector<LSFittedLine> fieldLines
        LinePoint leftVisualCalculated,rightVisualCalculated;
        Vector3<float> leftWMPolarPoint, rightWMPolarPoint;
        LinePoint leftWMPoint, rightWMPoint;

        LSFittedLine currentVisualLine = fieldLines[i];
        //Get 2 points corresponding to the end points on the line:
        //Check if vertical: FindXfromY, if not FindYfromX
        if(currentVisualLine.isVertical())
        {
            leftVisualCalculated.y = currentVisualLine.leftPoint.y;
            leftVisualCalculated.x = currentVisualLine.findXFromY(leftVisualCalculated.y);
            rightVisualCalculated.y = currentVisualLine.rightPoint.y;
            rightVisualCalculated.x = currentVisualLine.findXFromY(rightVisualCalculated.y);
        }
        else
        {
            leftVisualCalculated.x = currentVisualLine.leftPoint.x;
            leftVisualCalculated.y = currentVisualLine.findYFromX(leftVisualCalculated.x);
            rightVisualCalculated.x = currentVisualLine.rightPoint.x;
            rightVisualCalculated.y = currentVisualLine.findYFromX(rightVisualCalculated.x);
        }
        //USE Distance to point to transform the points to relative WM space:

        bool isOKA = GetDistanceToPoint(leftVisualCalculated, leftWMPolarPoint, vision);
        bool isOKB = GetDistanceToPoint(rightVisualCalculated, rightWMPolarPoint, vision);
        if(isOKA== false || isOKB ==false)
        {
            #if DEBUG_VISION_VERBOSITY > 0
                debug << "TransformLinesToWorldModelSpace:: Distance to Point Failed." << std::endl;
            #endif
            #if TARGET_OS_IS_WINDOWS
                qDebug() << "TransformLinesToWorldModelSpace:: Distance to Point Failed." << std::endl;
            #endif
            break;
        }

        //Calculate X,Y Points from Polar Points:
        leftWMPoint.x = leftWMPolarPoint.x * cos(leftWMPolarPoint.y) * cos (leftWMPolarPoint.z);
        leftWMPoint.y = leftWMPolarPoint.x * sin(leftWMPolarPoint.y) * cos (leftWMPolarPoint.z);
        rightWMPoint.x = rightWMPolarPoint.x * cos(rightWMPolarPoint.y) * cos (rightWMPolarPoint.z);
        rightWMPoint.y = rightWMPolarPoint.x * sin(rightWMPolarPoint.y) * cos (rightWMPolarPoint.z);

        LSFittedLine relWMLine;

        relWMLine.addPoint(leftWMPoint);
        relWMLine.addPoint(rightWMPoint);

        #if TARGET_OS_IS_WINDOWS
        qDebug()    << "VisualLeftPoint: " << leftVisualCalculated.x << "," << leftVisualCalculated.y
                    << "VisualRightPoint: " << rightVisualCalculated.x << "," << rightVisualCalculated.y
                    << "\t LeftPoint: " << relWMLine.leftPoint.x << "," << relWMLine.leftPoint.y
                    << "RightPoint: " << relWMLine.rightPoint.x << "," << relWMLine.rightPoint.y
                    << " \t "<< i << ": " << relWMLine.getA() << "x + "
                        << relWMLine.getB() << " y = " << relWMLine.getC() << " \t Angle: "
                        << relWMLine.getAngle()*57.2957795 << " Degrees." << currentVisualLine.numPoints;
                //qDebug() << leftWMPoint.x << "," << leftWMPoint.y << "," << rightWMPoint.x << "," << rightWMPoint.y;
        #endif
        relWMLine.valid = currentVisualLine.valid;
        transformedFieldLines.push_back(relWMLine);



    }

    //Find Candidates for CentreCircle Lines:

    std::vector<unsigned int> usedLines;

    for(unsigned int i = 0 ; i < transformedFieldLines.size(); i++ )
    {
        //if(!transformedFieldLines[i].valid)
        //    continue;
        float lineLength = sqrt((transformedFieldLines[i].leftPoint.x-transformedFieldLines[i].rightPoint.x) * (transformedFieldLines[i].leftPoint.x-transformedFieldLines[i].rightPoint.x) + (transformedFieldLines[i].leftPoint.y-transformedFieldLines[i].rightPoint.y)* (transformedFieldLines[i].leftPoint.y-transformedFieldLines[i].rightPoint.y));
        if (lineLength > 60 )
        {
            //qDebug() << "Line too long" << lineLength;
            continue;
        }
        for(unsigned int j = i+1; j < transformedFieldLines.size(); j++)
        {
            //if(!transformedFieldLines[j].valid)
            //    continue;
            float lineLength = sqrt((transformedFieldLines[j].leftPoint.x-transformedFieldLines[j].rightPoint.x) * (transformedFieldLines[j].leftPoint.x-transformedFieldLines[j].rightPoint.x) + (transformedFieldLines[j].leftPoint.y-transformedFieldLines[j].rightPoint.y)*( transformedFieldLines[j].leftPoint.y-transformedFieldLines[j].rightPoint.y));
            if (lineLength > 60)
            {
                //qDebug() << "Line too long" << lineLength;
                continue;
            }
            float distance;
            float xtheta1 = transformedFieldLines[j].leftPoint.x-transformedFieldLines[i].rightPoint.x;
            float ytheta1 = transformedFieldLines[j].leftPoint.y-transformedFieldLines[i].rightPoint.y;
            float xtheta2 = transformedFieldLines[j].rightPoint.x-transformedFieldLines[i].rightPoint.x;
            float ytheta2 = transformedFieldLines[j].rightPoint.y-transformedFieldLines[i].rightPoint.y;
            float xtheta3 = transformedFieldLines[j].leftPoint.x-transformedFieldLines[i].leftPoint.x;
            float ytheta3 = transformedFieldLines[j].leftPoint.y-transformedFieldLines[i].leftPoint.y;
            float distance1 = sqrt(xtheta1*xtheta1 + ytheta1*ytheta1);
            float distance2 = sqrt(xtheta2*xtheta2 + ytheta2*ytheta2);
            float distance3 = sqrt(xtheta3*xtheta3 + ytheta3*ytheta3);
            if(distance1 > 30   && distance2 > 30 && distance3 > 30)
            {
                //qDebug() << "Points Not close enough";
                continue;
            }
            else
            {
                distance = distance1;
                if(distance > distance2 )
                {
                    distance = distance2;
                }
                if( distance > distance3 )
                {
                    distance = distance3;
                }
            }
            float angle =  fabs(transformedFieldLines[i].getAngle()*57.2957795 - transformedFieldLines[j].getAngle()*57.2957795);
            //qDebug() << i << " and " << j << ": " << angle << "degrees \t Distance: " << distance;
            if((angle > 110 && angle < 160 ) || (angle > 20 && angle < 70) )
            {
                //qDebug() <<"\tADDED: "<< i << " and " << j << ": " << angle << "degrees \t Distance: " << distance;
                //qDebug() << transformedFieldLines[i].leftPoint.x << ", " << transformedFieldLines[i].leftPoint.y << ", " << transformedFieldLines[i].rightPoint.x << ", " << transformedFieldLines[i].rightPoint.y << i;
                //qDebug() << transformedFieldLines[j].leftPoint.x << ", " << transformedFieldLines[j].leftPoint.y << ", " << transformedFieldLines[j].rightPoint.x << ", " << transformedFieldLines[j].rightPoint.y << j;
                bool iused = false;
                bool jused = false;
                for(unsigned int k = 0; k < usedLines.size(); k++)
                {
                    if(usedLines[k] == i)
                    {
                        iused =true;
                    }
                    if(usedLines[k] == j)
                    {
                        jused =true;
                    }
                }
                if(!iused)
                {
                    usedLines.push_back(i);
                    std::vector  <LinePoint*> linePts = fieldLines[i].getPoints();
                    for(int k = 0; k <  fieldLines[i].numPoints ; k++)
                    {
                        LinePoint* tempPoint = linePts.at(k);
                        centreCirclePoints.push_back(tempPoint);
                    }

                }
                if(!jused)
                {
                    usedLines.push_back(j);
                    std::vector  <LinePoint*> linePts = fieldLines[j].getPoints();
                    for(int k = 0; k <  fieldLines[j].numPoints ; k++)
                    {
                        LinePoint* tempPoint = linePts.at(k);
                        centreCirclePoints.push_back(tempPoint);
                    }
                }

            }
        }
    }
    // CentreCirclePoints:
    int sum=0;
    #if TARGET_OS_IS_WINDOWS
        qDebug() << "Centre Circle Points: Lines used " << usedLines.size();
    #endif
    for(unsigned int i = 0; i < usedLines.size(); i++)
    {
        #if TARGET_OS_IS_WINDOWS
            qDebug() << transformedFieldLines[usedLines[i]].leftPoint.x << ", " << transformedFieldLines[usedLines[i]].leftPoint.y << ", " << transformedFieldLines[usedLines[i]].rightPoint.x << ", " << transformedFieldLines[usedLines[i]].rightPoint.y << usedLines[i] << fieldLines[usedLines[i]].numPoints;
        #endif
        sum = sum + fieldLines[usedLines[i]].numPoints;
    }
    //qDebug() << "Number of Points: " << sum;
    for(unsigned int i = 0; i < usedLines.size(); i++)
    {
        //qDebug() << centreCirclePoints[i]->x <<"," << centreCirclePoints[i]->y;
        std::vector  <LinePoint*> linePts = fieldLines[usedLines[i]].getPoints();
        for(int k = 0; k <  fieldLines[usedLines[i]].numPoints ; k++)
        {
            LinePoint* tempPoint = linePts.at(k);
            if(tempPoint->inUse == false)
                continue;
            Vector3<float> point;
            bool isok = GetDistanceToPoint(*tempPoint, point, vision);
            if(isok)
            {
                #if TARGET_OS_IS_WINDOWS
                    float x = point.x * cos(point.y) * cos (point.z);
                    float y = point.x * sin(point.y) * cos (point.z);
                    qDebug() << x <<"," << y <<"," << usedLines[i];
                #endif
            }
        }

    }
}


/*------------------
// Method: 	FindCornerPoints
// Arguments: 	void
// Returns: 	Void
// Description: Finds cornerpoints from lines that intersect
//		Stores corner points in global variable "cornerPoints".
------------------*/

void LineDetection::FindCornerPoints(int IMAGE_WIDTH,int IMAGE_HEIGHT, Vision* vision){

	//std::cout<<"Starting to Look for Corners.."<< std::endl;
	if (TotalValidLines <2)
	{
                //qDebug("%i Valid Lines... ABORT\n",TotalValidLines);
		return;
	}
	//else
                //qDebug("TotalValidLines: %i\n",TotalValidLines);
  
	int CommonX, CommonY;
	unsigned short Top,Bottom,Left,Right,Type;
  
  //Now try and find where the lines intersect.
        for (unsigned int LineIDStart = 0; LineIDStart < fieldLines.size()-1; LineIDStart++){
                 if(cornerPoints.size()> MAX_CORNERPOINTS) break;
		if (!fieldLines[LineIDStart].valid) continue;
		//See if this line intersects with any other ones...
                for (unsigned int LineIDCheck = LineIDStart+1; LineIDCheck < fieldLines.size(); LineIDCheck++){
			if (!fieldLines[LineIDCheck].valid) continue;
                        //Old Check this lines' slops are far enough apart...
                        //if (!(fabs(fieldLines[LineIDStart].getAngle() - fieldLines[LineIDCheck].getAngle()) >=.08)) continue;
                        //New Check: Transformed Line must be about 90 degrees
                        if ((fabs(fabs((transformedFieldLines[LineIDStart].getAngle() - transformedFieldLines[LineIDCheck].getAngle())*57.2957795)-90) > 25)) continue;
			//std::cout << "Comparing Line Angles: " << LineIDStart << ", " << LineIDCheck<< std::endl;
			//this seems to be very oblique?? make more acute for circle stuff ALEX
			//Work out their intersecting X pos..
			CommonX = 	(int)((fieldLines[LineIDStart].getYIntercept() - fieldLines[LineIDCheck].getYIntercept()) / 
					(fieldLines[LineIDCheck].getGradient() - fieldLines[LineIDStart].getGradient()));
			//Check if they intersect on the screen.. (or near enough..)
			if (!(CommonX > 0 && CommonX < IMAGE_WIDTH)) continue;
			//This should be on the screen, so let's work out the Y co-ords;
			//It doesn't matter which line we use to call the point, however for ease, use the one with the best slope...
			if (fieldLines[LineIDStart].getGradient() < 1 || fieldLines[LineIDStart].getGradient() > -1)
				CommonY = (int)((fieldLines[LineIDStart].getGradient()*CommonX) + fieldLines[LineIDStart].getYIntercept());
			else 
				CommonY = (int)((fieldLines[LineIDCheck].getGradient()*CommonX) + fieldLines[LineIDCheck].getYIntercept());
			
			if (!(CommonY > 0 && CommonY < IMAGE_HEIGHT))continue;
			//This should be a good point on the screen!!!
                        //if (!(cornerPoints.size() < MAX_CORNERPOINTS))continue;

			//std::cout << "Common Points: " << CommonX << "," << CommonY<< std::endl;
                        CornerPoint tempCornerPoint;
                        tempCornerPoint.PosX = CommonX;
                        tempCornerPoint.PosY = CommonY;

			// use this stuff for decoding corners:    ALEX
                        tempCornerPoint.Line[0] = fieldLines[LineIDStart];
                        tempCornerPoint.Line[1] = fieldLines[LineIDCheck];
			
                        tempCornerPoint.Direction = 0;
                        tempCornerPoint.Orientation = 0;
                        tempCornerPoint.CornerType = 0;
                        tempCornerPoint.FieldObjectID = 0;

			//Work out what kind of a corner this is and what way it's facing...
			Top=Bottom=Left=Right=0;
			for (int x = 0; x <=1; x++){
				Type = 0;
				//Find the MaxX MinX MaxY MinY of each line...
                                LSFittedLine tempLine = tempCornerPoint.Line[x];
				int minX, minY, maxX, maxY;
                                minX = (int)tempLine.leftPoint.x;
                                maxX = (int)tempLine.rightPoint.x;
				//Check if the Y values(not in order):
				if (tempLine.leftPoint.y > tempLine.rightPoint.y){
                                        minY = (int)tempLine.rightPoint.y;
                                        maxY = (int)tempLine.leftPoint.y;
				}
				else{
                                        maxY = (int)tempLine.rightPoint.y;
                                        minY = (int)tempLine.leftPoint.y;
				}

				// the '5' used here should be defined or enumerated. likely will need adjustment with new camera resolution. double it? ALEX
				if (minY < (CommonY-5)){
					Top++;
					Type++;
					if (maxY > CommonY+5){
						//If it's both, this must be noted as special...
						Type+= 2;
						Bottom++;
					}
				}
				else if (maxY > CommonY+5){
					Bottom++;
					Type++;
				}
				if (minX < CommonX-5){
					Left++;
					Type++;
					if (maxX > CommonX+5){
					//If it's both, this must be noted as special...
					Type+=2;
					Right++;
					}
				}
				else if (maxX > CommonX+5){
					Right++;
					Type++;
				}
				if (Type > 2){ 
                                        tempCornerPoint.CornerType++;
				}
			}
			//Now work out the corner's details...
	
			// *******    Where is this orientation data used???? ALEX    ********
	
			//cornerPoints[CornerPointCounter]->CornerType = 4;
			
                        if (tempCornerPoint.CornerType == 0){
			//L shape corner..
				if (Left == Right){
					if(Bottom >= Top)
                                        {
                                                tempCornerPoint.Orientation = POINT_UP; // was -1 is 1
                                                //qDebug() << "L Corner: UP \t" << tempCornerPoint.PosX << "," << tempCornerPoint.PosY;
                                        }
					else
                                        {
                                                tempCornerPoint.Orientation = POINT_DOWN;  // was 1 is 2
                                                //qDebug() << "L Corner: DOWN \t " << tempCornerPoint.PosX << "," << tempCornerPoint.PosY;
                                        }
				}
				else {
					if (Left >= Right)
                                        {
                                                tempCornerPoint.Orientation = POINT_RIGHT;  //4
                                                //qDebug() << "L Corner: RIGHT \t " << tempCornerPoint.PosX << "," << tempCornerPoint.PosY;
                                        }
					else 
                                        {
                                                tempCornerPoint.Orientation = POINT_LEFT;  //3
                                                //qDebug() << "L Corner: LEFT \t " << tempCornerPoint.PosX << "," << tempCornerPoint.PosY;
                                        }
				}
			}
                        if (tempCornerPoint.CornerType == 1){
			//T shape corner..
				if (Top >= Bottom && Right >= Left)
                                        tempCornerPoint.Orientation = OUT_BOUNDS_VIEW_RIGHT; //1 //false positives occour in center cross
				else{
					if (Top >= Bottom && Left <= Right) {
                                                tempCornerPoint.Orientation = OUT_BOUNDS_VIEW_LEFT;  //2  //false positives occour in center cross
					}
					else {
						if (Bottom >= Top && Right >= Left)
                                                        tempCornerPoint.Orientation = POINT_UP_AND_LEFT;  //3
						else 
                                                        tempCornerPoint.Orientation = POINT_UP_AND_RIGHT;  //4
					}
				}
			}

                        float angle = findAngleOfLCorner(tempCornerPoint);
                        //qDebug() << "Angle: " << angle*57.2957795 << "Degrees";
                        if(angle > 3.14/4)
                        {
                            tempCornerPoint.isObtuse = true;
                        }
                        else
                        {
                            tempCornerPoint.isObtuse = false;
                        }


                        if(DetectWhitePixels(tempCornerPoint.PosX,tempCornerPoint.PosY, 2,vision))
                        {
                            cornerPoints.push_back(tempCornerPoint);
                        }

		}
	}
        //qDebug() << "Total Corners Found: " << cornerPoints.size();

        //for (unsigned int i = 0; i < cornerPoints.size() ; i++)
        //{
            //qDebug() << i << ": \t "<< cornerPoints[i].PosX << ","<< cornerPoints[i].PosY;
        //}
}


//Find the Angle between the L corners:

float LineDetection::findAngleOfLCorner(CornerPoint cornerPoint)
{
        float angle = 0.0;
        LSFittedLine Line1 = cornerPoint.Line[0];
        LSFittedLine Line2 = cornerPoint.Line[1];

        //Get Unit Vector:
        Point L1Left = Line1.leftPoint;
        Point L2Left = Line2.leftPoint;

        float LengthV1 = sqrt((L1Left.x - cornerPoint.PosX)*(L1Left.x - cornerPoint.PosX) + (L1Left.y - cornerPoint.PosY)*(L1Left.y - cornerPoint.PosY));
        float LengthV2 = sqrt((L2Left.x - cornerPoint.PosX)*(L2Left.x - cornerPoint.PosX) + (L2Left.y - cornerPoint.PosY)*(L2Left.y - cornerPoint.PosY));
        Vector2<float> V1, V2;
        V1.x = (L1Left.x - cornerPoint.PosX) / (LengthV1);
        V1.y = (L1Left.y - cornerPoint.PosY) / (LengthV1);

        V2.x = (L2Left.x - cornerPoint.PosX) / (LengthV2);
        V2.y = (L2Left.y - cornerPoint.PosY) / (LengthV2);

        angle = acos(V1.x*V2.x + V1.y *V2.y);

        return angle;
}

/*------------------
// Method: 	DecodeCorners
// Arguments: 	FieldObjects* AllObjects, float timestamp
// Returns: 	Void
// Description: Assigns known corner poitns to field objects
------------------*/



void LineDetection::DecodeCorners(FieldObjects* AllObjects, double timestamp, Vision* vision)
{
	
    double TempDist = 0.0;
    double TempBearing = 0.0;
    double TempElev = 0.0;
    int TempID;
    unsigned int x;
    bool recheck = false;

    int cornerPointsOnScreen = 0;
    for(unsigned int i = 0; i < cornerPoints.size(); i++)
    {
        if(vision->isPixelOnScreen(cornerPoints[i].PosX,cornerPoints[i].PosY) )
        {
            cornerPointsOnScreen++;
        }
    }
    //Check if any goals are close by that has been seen in current image:
    bool closeGoalSeen = false;
    float closeGoalDistance = 10000;
    for(unsigned int i = FieldObjects::FO_BLUE_LEFT_GOALPOST; i <= FieldObjects::FO_YELLOW_RIGHT_GOALPOST; i++)
    {
        if(AllObjects->stationaryFieldObjects[i].TimeLastSeen() == timestamp)
        {

            if(closeGoalDistance > AllObjects->stationaryFieldObjects[i].measuredDistance())
            {
                closeGoalDistance = AllObjects->stationaryFieldObjects[i].measuredDistance();
            }
            if(AllObjects->stationaryFieldObjects[i].measuredDistance() < 300)
            {
                closeGoalSeen = true;

            }
        }
    }

    for(unsigned int i = 0; i < AllObjects->ambiguousFieldObjects.size(); i++)
    {
        if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN
           || AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
        {
            if(closeGoalDistance > AllObjects->ambiguousFieldObjects[i].measuredDistance())
            {
                closeGoalDistance = AllObjects->ambiguousFieldObjects[i].measuredDistance();
            }
            if(AllObjects->ambiguousFieldObjects[i].measuredDistance() < 300)
            {
                closeGoalSeen = true;

            }
        }
    }

    //********  this filters out center circle. only a count 0f 2 is checked.
    /*if ( (cornerPoints.size() > 8 || cornerPointsOnScreen > 5) && closeGoalSeen == false )
    {
        //PERFORM ELIPSE FIT HERE!

        //Method:
        //	1. Box all small lines:
        //	2. find the most left line: use right point
        // 	3. search for next line by looking for a left point that is close to the right point
        //	4. fill points array
        //	5. Repeat until no more further right

        //TRY BOX around all small Lines and compute centre of box

        // Find shortest Lines
        //int cutOff = IMAGE_WIDTH/2;
        int longestLine = 0;
        double longestLineLength = 0.0;


        #if TARGET_OS_IS_WINDOWS
            qDebug()  << "Start Centre Circle Detection: Corners " << cornerPoints.size() << cornerPointsOnScreen << std::endl;
        #endif
        #if DEBUG_VISION_VERBOSITY > 0
            debug  << "Start Centre Circle Detection: Corners " << cornerPoints.size() << cornerPointsOnScreen <<std::endl;
        #endif

        //Sort Lines by most Left:
        qsort(fieldLines, 0, fieldLines.size()-1);

        for (unsigned int i = 0; i<fieldLines.size(); i++)
        {
            //std::cout << "i:";
            if(fieldLines[i].valid == false)	continue;
            int lx,ly,rx,ry;
            lx = fieldLines[i].leftPoint.x;
            ly = fieldLines[i].leftPoint.y;
            rx = fieldLines[i].rightPoint.x;
            ry = fieldLines[i].rightPoint.y;

            double linelength = sqrt((lx-rx)*(lx-rx) + (ly-ry)*(ly-ry));
            if(linelength > longestLineLength)
            {
                longestLine = i;
                longestLineLength = linelength;
            }
        }
        //Find Points:

        std::vector<LinePoint*> points;
        for(unsigned int i = 0; i < fieldLines.size(); i++)
        {

            if ( i == (unsigned int) PenaltySpotLineNumber)	continue;
            if ( i == (unsigned int) longestLine )		continue;
            if (fieldLines[i].valid == false)	continue;

            //std::vector<LinePoint*> minpoints = fieldLines[i].getPoints();

            //Find another line thats close to it
            int lrX = fieldLines[i].rightPoint.x;
            int lrY = fieldLines[i].rightPoint.y;
            bool hasJoinedLines = false;
            for(unsigned int j = i; j < fieldLines.size(); j++)
            {
                if ( j == (unsigned int) PenaltySpotLineNumber)	continue;
                if ( j == (unsigned int) longestLine )			continue;
                if (fieldLines[j].valid == false)	continue;
                if (i == j)				continue;

                //Calculate distance from 2 points
                int rrX = fieldLines[j].leftPoint.x;
                int rrY = fieldLines[j].leftPoint.y;
                double distance = sqrt((rrX-lrX)*(rrX-lrX) + (rrY-lrY)*(rrY-lrY));
                if(distance < 32 && distance >= 0 && rrX > lrX)
                {

                    hasJoinedLines = true;
                    std::vector <LinePoint*> linePts = fieldLines[j].getPoints();
                    for(int k = 0; k <  fieldLines[j].numPoints ; k++)
                    {
                        LinePoint* tempPoint = linePts.at(k);
                        points.push_back(tempPoint);
                    }
                    lrX = fieldLines[j].rightPoint.x;
                    lrY = fieldLines[j].rightPoint.y;
                    break;
                }
            }
            if(hasJoinedLines == true)
            {
                std::vector  <LinePoint*> linePts = fieldLines[i].getPoints();
                for(int k = 0; k <  fieldLines[i].numPoints ; k++)
                {
                    LinePoint* tempPoint = linePts.at(k);
                    points.push_back(tempPoint);
                }
            }
            if(points.size() > 50)
            {
                break;
            }
        }
        #if TARGET_OS_IS_WINDOWS
            qDebug()  << "LinePoints For Centre Circle Found: " << points.size()<< std::endl;
        #endif
        #if DEBUG_VISION_VERBOSITY > 0
            debug  << "LinePoints For Centre Circle Found: " << points.size()<< std::endl;
        #endif

        if(points.size() > 5)
        {
           
            FitEllipseThroughCircle ellipseCircleFitter;
            bool isOK = ellipseCircleFitter.Fit_Ellipse_Through_Circle(centreCirclePoints, vision);
            #if TARGET_OS_IS_WINDOWS
                qDebug() << "Ellipse Results: "<< isOK << ellipseCircleFitter.relCx <<  ellipseCircleFitter.relCy << ellipseCircleFitter.r;
            #endif
            #if DEBUG_VISION_VERBOSITY > 0
                debug << "Ellipse Results: "<< isOK << ellipseCircleFitter.relDistance <<  ellipseCircleFitter.relBearing << ellipseCircleFitter.r;
            #endif


            if(isOK  == false)
            {
                
                
                double cx =0;
                double cy =0;
                double r1 =0;
                double r2 =0;
                EllipseFit* e = new EllipseFit;

                e->Fit_Ellipse(points);
                //e->PrintFinal();
                cx = e->GetX();
                cy = e->GetY();
                r1 = e->GetR1();
                r2 = e->GetR2();

                TempDist = 0.0;
                Vector2<float> screenPositionAngle((float)vision->CalculateBearing(cx), (float)vision->CalculateElevation(cy));
                GetDistanceToPoint(cx, cy, &TempDist, &TempBearing, &TempElev, vision);
                #if TARGET_OS_IS_WINDOWS
                    qDebug() << TempDist << closeGoalDistance <<  fabs( TempDist - closeGoalDistance);
                #endif

                if (TempDist > 100.0  && TempDist != 0.0  && fabs( TempDist - closeGoalDistance) > 200) {

                    Vector3<float> measured((float)TempDist,(float)TempBearing,(float)TempElev);
                    Vector3<float> measuredError(0.0,0.0,0.0);
                    Vector2<int> screenPosition(cx, cy);
                    Vector2<int> sizeOnScreen;
                    if(r2 > r1)
                    {
                       sizeOnScreen.x = r2*2;
                       sizeOnScreen.y = r1*2;
                    }
                    else
                    {
                        sizeOnScreen.x = r1*2;
                        sizeOnScreen.y = r2*2;
                    }

                    AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].UpdateVisualObject(measured,measuredError,screenPositionAngle,screenPosition,sizeOnScreen,timestamp);
                    return;
                }
                
            }
            else
            {

                #if TARGET_OS_IS_WINDOWS
                    qDebug()  << "Ellipse Fit Through Circle: [" << ellipseCircleFitter.relCx << ", " << ellipseCircleFitter.relCy << ", "<< ellipseCircleFitter.r << "]"<<std::endl;
                #endif
                #if DEBUG_VISION_VERBOSITY > 5
                    debug  << "Ellipse Fit Through Circle: [" << ellipseCircleFitter.cx << ", " << ellipseCircleFitter.cy << ", "<< "]"<<std::endl;
                #endif
                Vector3<float> measured((float)ellipseCircleFitter.relDistance,(float)ellipseCircleFitter.relBearing, (float)ellipseCircleFitter.relElevation);
                Vector3<float> measuredError(ellipseCircleFitter.sd,0.0,0.0);
                Vector2<int> screenPosition(round(ellipseCircleFitter.cx), round(ellipseCircleFitter.cy));
                Vector2<float> screenPositionAngle((float)vision->CalculateBearing(ellipseCircleFitter.cx), (float)vision->CalculateElevation(ellipseCircleFitter.cy));
                Vector2<int> sizeOnScreen;
                if(ellipseCircleFitter.r2 > ellipseCircleFitter.r1)
                {
                   sizeOnScreen.x = round(ellipseCircleFitter.r2);
                   sizeOnScreen.y = round(ellipseCircleFitter.r1);
                }
                else
                {
                    sizeOnScreen.x = round(ellipseCircleFitter.r1);
                    sizeOnScreen.y = round(ellipseCircleFitter.r2);
                }

                AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].UpdateVisualObject(measured,measuredError,screenPositionAngle,screenPosition,sizeOnScreen,vision->m_timestamp);
                
                return;
            }
            //qDebug() << "Center Circle: " << cx << "," << cy <<std::endl;


        }

    }
    */
    if(centreCirclePoints.size() > 10 && closeGoalSeen == false)
    {
        FitEllipseThroughCircle ellipseCircleFitter;
        bool isOK = ellipseCircleFitter.Fit_Ellipse_Through_Circle(centreCirclePoints, vision);
        #if TARGET_OS_IS_WINDOWS
            qDebug() << "Ellipse Results: "<< isOK << ellipseCircleFitter.relCx <<  ellipseCircleFitter.relCy << ellipseCircleFitter.r;
        #endif
        #if DEBUG_VISION_VERBOSITY > 0
            debug << "Ellipse Results: "<< isOK << ellipseCircleFitter.relDistance <<  ellipseCircleFitter.relBearing << ellipseCircleFitter.r;
        #endif


        if(isOK  == true)
        {

            #if TARGET_OS_IS_WINDOWS
                qDebug()  << "Ellipse Fit Through Circle: [" << ellipseCircleFitter.relCx << ", " << ellipseCircleFitter.relCy << ", "<< ellipseCircleFitter.r << "]"<<std::endl;
            #endif
            #if DEBUG_VISION_VERBOSITY > 5
                debug  << "Ellipse Fit Through Circle: [" << ellipseCircleFitter.cx << ", " << ellipseCircleFitter.cy << ", "<< "]"<<std::endl;
            #endif
            Vector3<float> measured((float)ellipseCircleFitter.relDistance,(float)ellipseCircleFitter.relBearing, (float)ellipseCircleFitter.relElevation);
            Vector3<float> measuredError(ellipseCircleFitter.sd,0.0,0.0);
            Vector2<int> screenPosition(round(ellipseCircleFitter.cx), round(ellipseCircleFitter.cy));
            Vector2<float> screenPositionAngle((float)vision->CalculateBearing(ellipseCircleFitter.cx), (float)vision->CalculateElevation(ellipseCircleFitter.cy));
            Vector2<int> sizeOnScreen;
            if(ellipseCircleFitter.r2 > ellipseCircleFitter.r1)
            {
               sizeOnScreen.x = round(ellipseCircleFitter.r2);
               sizeOnScreen.y = round(ellipseCircleFitter.r1);
            }
            else
            {
                sizeOnScreen.x = round(ellipseCircleFitter.r1);
                sizeOnScreen.y = round(ellipseCircleFitter.r2);
            }

            if(fabs(closeGoalDistance - ellipseCircleFitter.relDistance) > 200)
            {
                AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].UpdateVisualObject(measured,measuredError,screenPositionAngle,screenPosition,sizeOnScreen,vision->m_timestamp);
            }
        }
    }

        //qDebug() << "Before Decoding Lines: ";
        /*
        for(unsigned int i = 0; i < AllObjects->stationaryFieldObjects.size();i++)
        {
            if(AllObjects->stationaryFieldObjects[i].isObjectVisible() == true)
            {
                //qDebug() << "Stationary Object: " << i << ":" //<<AllObjects->stationaryFieldObjects[i].getName()
                         //<<"Seen at "<< AllObjects->stationaryFieldObjects[i].ScreenX()
                         //<<","       << AllObjects->stationaryFieldObjects[i].ScreenY()
                        //<< "\t Distance: " <<AllObjects->stationaryFieldObjects[i].measuredDistance();
            }
        }
        for(unsigned  int i = 0; i <AllObjects->mobileFieldObjects.size();i++)
        {
            if(AllObjects->mobileFieldObjects[i].isObjectVisible() == true)
            {
                //qDebug() << "Mobile Object: " << i << ":" //<<AllObjects->mobileFieldObjects[i].getName()
                         //<< "Seen at "   << AllObjects->mobileFieldObjects[i].ScreenX()
                         //<<","           << AllObjects->mobileFieldObjects[i].ScreenY()
                        //<< "\t Distance: " <<AllObjects->mobileFieldObjects[i].measuredDistance();
            }
        }

        for(unsigned int i = 0; i <AllObjects->ambiguousFieldObjects.size();i++)
        {
            if(AllObjects->ambiguousFieldObjects[i].isObjectVisible() == true)
            {
                //qDebug() << "Ambiguous Object: " << i << ":" <<AllObjects->ambiguousFieldObjects[i].getID()
                //         << "Seen at "          << AllObjects->ambiguousFieldObjects[i].ScreenX()
                //         << ","                 << AllObjects->ambiguousFieldObjects[i].ScreenY()
                 //        << "\t Distance: " <<AllObjects->ambiguousFieldObjects[i].measuredDistance()
                 //        << AllObjects->ambiguousFieldObjects[i].isObjectVisible();

            }
        }
        */
        bool CheckedCornerPoints[cornerPoints.size()];
        for(unsigned int i = 0; i < cornerPoints.size(); i++)
        {
            CheckedCornerPoints[i] = false;
        }
	//CHECK EACH CORNER POINT:	
        for (x = 0; x < cornerPoints.size(); x++)
        {
                if(recheck ==true)
                {
                    recheck =false;
                    x = 0;
                    //qDebug("Resetting X to perform Recheck.... \n");
                }
                if(CheckedCornerPoints[x] == true) continue;
                TempID = 0;
                //qDebug("Checking CornerID: %i \t",x);
		//ASSIGNING T, L or X corner To TempID
		if (cornerPoints[x].CornerType == 0)
                {
                    if(cornerPoints[x].Orientation == POINT_UP)
                    {
                        TempID = FieldObjects::FO_CORNER_UNKNOWN_INSIDE_L;
                    }
                    else
                    {
                        TempID = FieldObjects::FO_CORNER_UNKNOWN_OUTSIDE_L;
                    }
                    //qDebug("FO_CORNER_UNKNOWN_L located \n");
                }
		else if (cornerPoints[x].CornerType == 1) //create new type for cross and check here. add definition in globals FO_CORNER_UNKNOWN_X
                {
                    TempID = FieldObjects::FO_CORNER_UNKNOWN_T;
                    //qDebug("FO_CORNER_UNKNOWN_T located \n");
                }
                else if (cornerPoints[x].CornerType > 1)
                {
                    //qDebug("FO_CORNER_UNKNOWN_X located \n");
                    continue;
                }
		
		//START DECODING T
		if (TempID){
			//Initialising Variables
                        Vector2<float> screenPositionAngle(vision->CalculateBearing(cornerPoints[x].PosX), vision->CalculateElevation(cornerPoints[x].PosY));
                        GetDistanceToPoint(cornerPoints[x].PosX, cornerPoints[x].PosY, &TempDist, &TempBearing, &TempElev, vision);
                        //qDebug() << "Corner " << x << ": " <<  TempDist;
                        if(TempDist > 800)
                        {
                            CheckedCornerPoints[x] = true;
                            continue;
                        }
                        AmbiguousObject tempUnknownCorner(TempID, "Unknown Corner");
                        Vector3<float> measured((float)TempDist,(float)TempBearing,(float)TempElev);
                        Vector3<float> measuredError(0.0,0.0,0.0);
                        Vector2<int> screenPosition(cornerPoints[x].PosX, cornerPoints[x].PosY);
                        Vector2<int> sizeOnScreen(4,4);
                        tempUnknownCorner.UpdateVisualObject(measured,measuredError,screenPositionAngle,screenPosition,sizeOnScreen,timestamp);

                        //fieldObjects[TempID].visionBearing = TempBearing;
                        //fieldObjects[TempID].visionDistance = TempDist;
                        //fieldObjects[TempID].visionElevation = TempElev;
                        //fieldObjects[TempID].visionX = cornerPoints[x].PosX;
                        //fieldObjects[TempID].visionY = cornerPoints[x].PosY;

			//--------------Goal T and goal post combo rule:----------------------------------------------------------------------------------
			if (cornerPoints[x].CornerType == 1){  // must include a half field range limit because of false positives on center circle
				if((cornerPoints[x].Orientation == 3) || (cornerPoints[x].Orientation == 4)){ // could divide limit by distance to get a range adjustment		
                                    for(unsigned int FO_Counter = 0; FO_Counter < AllObjects->ambiguousFieldObjects.size(); FO_Counter++ )
                                    {
                                        if(AllObjects->ambiguousFieldObjects[FO_Counter].getID() != FieldObjects::FO_BLUE_GOALPOST_UNKNOWN
                                           || AllObjects->ambiguousFieldObjects[FO_Counter].isObjectVisible() == false) continue;
                                        if( 	( (fabs((AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX()) - (cornerPoints[x].PosX))) < POST_T_LIMIT )
                                                && (AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance() < 350)
                                                && (tempUnknownCorner.isObjectVisible() == true)){

                                                if( ( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) > 0
                                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() == false
                                                    || AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen())                                                     ){
                                                        //qDebug("\nTARGET ACQUIRED, BLUE left goal T       ..u\n");

							//COPY: fieldObjects[TempID] TO fieldObjects[FO_CORNER_BLUE_T_LEFT]
                                                        if(fabs(tempUnknownCorner.measuredDistance() - AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                                        {
                                                            recheck = true;
                                                            AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_T_LEFT].CopyObject(tempUnknownCorner);
                                                            tempUnknownCorner.setVisibility(false);
                                                        }
							//COPY: fieldObjects[FO_BLUE_GOALPOST_UNKNOWN] TO fieldObjects[FO_BLUE_LEFT_GOALPOST]

                                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                                        //Remove Unknown GoalPost:
                                                        AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);

						}
                                                else if( ( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) < 0
                                                         && ( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible() == false
                                                         || AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen() ) )
                                                {
                                                        //qDebug("\nTARGET ACQUIRED, BLUE right goal T       ..u\n");
                                                    if(fabs(tempUnknownCorner.measuredDistance() - AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                                    {
							recheck = true;
							////COPY: fieldObjects[TempID] TO fieldObjects[FO_CORNER_BLUE_T_RIGHT]
                                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_T_RIGHT].CopyObject(tempUnknownCorner);
                                                        tempUnknownCorner.setVisibility(false);
                                                    }
							//COPY: fieldObjects[FO_BLUE_GOALPOST_UNKNOWN] TO fieldObjects[FO_BLUE_RIGHT_GOALPOST]
                                                    AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                                    AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);
						}
					}
                                    }
                                    if( 	(AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() == true)
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                        && (tempUnknownCorner.isObjectVisible() == true)
                                        && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].ScreenX()- cornerPoints[x].PosX ) < POST_T_LIMIT )
                                        && (( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) > 0 )
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance()< 350))
                                    {
                                        //qDebug("\nTARGET ACQUIRED, BLUE left goal T      ..\n");
                                        //COPY: fieldObjects[TempID] TO fieldObjects[FO_CORNER_BLUE_T_LEFT]
                                        if(fabs(tempUnknownCorner.measuredDistance() - AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                        {
                                            AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_T_LEFT].CopyObject(tempUnknownCorner);
                                            tempUnknownCorner.setVisibility(false);
                                        }
                                    }

                                    else if( 	(AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible() == true)
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                        && (tempUnknownCorner.isObjectVisible() == true)
                                        && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) < POST_T_LIMIT )
                                        && (( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) < 0 )
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance() < 350))
                                    {

                                        //qDebug("\nTARGET ACQUIRED,  BLUE right goal T       ..\n");
                                        if(fabs(tempUnknownCorner.measuredDistance() - AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                        {
                                            AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_T_RIGHT].CopyObject(tempUnknownCorner);
                                            tempUnknownCorner.setVisibility(false);
                                        }
                                    }

				}// end if((cornerPoints[x]->Orientation == 3)........
			} // end if (cornerPoints[x]->CornerType == 1)

			//-----------end Goal T and goal post combo rule ----------------------------------------------------------------------------------


			//--------------penalty L and goal post combo rule:--------------------------------------------------------------------------------
			// must include a half field range limit because of false positives on center circle
                        if ((cornerPoints[x].CornerType == 0))
                        {

                            if (    ((cornerPoints[x].Orientation == 4)||(cornerPoints[x].Orientation == 2))
                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() == true)
                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                    && (tempUnknownCorner.isObjectVisible() == true)
                                    && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) < POST_L_LIMIT)
                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance() < 350)) {

                                    //qDebug("\nTARGET ACQUIRED, BLUE left penalty L       =\n");
                                if(tempUnknownCorner.measuredDistance()!= 0 && fabs(tempUnknownCorner.measuredDistance() - AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                {
                                    AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_PEN_LEFT].CopyObject(tempUnknownCorner);
                                    tempUnknownCorner.setVisibility(false);
                                }
                            }

                            else if (     ((cornerPoints[x].Orientation == 3)||(cornerPoints[x].Orientation == 2))
                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible() == true)
                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                    && (tempUnknownCorner.isObjectVisible() == true)
                                    && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) < POST_L_LIMIT)
                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance() < 350)) {

                                    //qDebug("\nTARGET ACQUIRED, BLUE right penalty L       =\n");
                                if( tempUnknownCorner.measuredDistance()!= 0 && fabs(tempUnknownCorner.measuredDistance() - AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                {
                                    AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_PEN_RIGHT].CopyObject(tempUnknownCorner);
                                    tempUnknownCorner.setVisibility(false);
                                }
                            }
                            for(unsigned int FO_Counter = 0; FO_Counter < AllObjects->ambiguousFieldObjects.size(); FO_Counter++ )
                            {
                                if(AllObjects->ambiguousFieldObjects[FO_Counter].getID() != FieldObjects::FO_BLUE_GOALPOST_UNKNOWN
                                   || AllObjects->ambiguousFieldObjects[FO_Counter].isObjectVisible() == false) continue;

                                if (    (cornerPoints[x].Orientation == 4)
                                        && (tempUnknownCorner.isObjectVisible() == true)
                                        && ( AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() == false
                                            || (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen()) )
                                        && (fabs( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX()- cornerPoints[x].PosX ) < POST_L_LIMIT)
                                        && (( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX()- cornerPoints[x].PosX ) > 0)
                                        && (AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance()< 350)) {

                                        //qDebug("\nTARGET ACQUIRED, BLUE left penalty L       u=\n");
                                        if(tempUnknownCorner.measuredDistance() != 0 && fabs(tempUnknownCorner.measuredDistance() - AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                        {
                                            recheck = true;
                                            AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_PEN_LEFT].CopyObject(tempUnknownCorner);
                                            tempUnknownCorner.setVisibility(false);
                                        }
                                        // blue left post should be populated with unknown data here.
                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                        AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);
                                }

                                else if (    (cornerPoints[x].Orientation == 3)
                                        && (tempUnknownCorner.isObjectVisible()== true)
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible() == false
                                        || (AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen()) )
                                        && (fabs( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) < POST_L_LIMIT)
                                        && (( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) < 0)
                                        && (AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance() < 350)) {

                                    //qDebug("\nTARGET ACQUIRED, BLUE right penalty L      u=\n");
                                    if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() - AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                    {
                                        recheck = true;
                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_BLUE_PEN_RIGHT].CopyObject(tempUnknownCorner);
                                        tempUnknownCorner.setVisibility(false);
                                    }

                                    // blue right post should be populated with unknown data here.
                                    AllObjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                    AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);
                                }
                            }
                        }
                                                //-------------- end penalty L and goal post combo rule ---------------------------------------------------------------------------
			//////////////////////////////////////////////////// END BLUE Goal ////////////////////////////////////////////////////////////////

			//--------------Goal T and goal post combo rule:----------------------------------------------------------------------------------
			if (cornerPoints[x].CornerType == 1){  // should include a half field range limit because of false positives on center circle
                            if((cornerPoints[x].Orientation == 3) || (cornerPoints[x].Orientation == 4)){// could divide limit by distance to get a range adjustment
                                for(unsigned int FO_Counter = 0; FO_Counter < AllObjects->ambiguousFieldObjects.size(); FO_Counter++)
                                {

                                    if(AllObjects->ambiguousFieldObjects[FO_Counter].getID() != FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN
                                       || AllObjects->ambiguousFieldObjects[FO_Counter].isObjectVisible() == false) continue;

                                    if( (tempUnknownCorner.isObjectVisible() == true)
                                        && ( (fabs((AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX()) - (cornerPoints[x].PosX))) < POST_T_LIMIT )
                                        && (AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance() < 350))
                                        {

                                        if( ( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) > 0
                                            && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() == false
                                                || AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen()))
                                        {
                                            //qDebug("\nTARGET ACQUIRED, YELLOW left goal T       ..u\n");
                                            if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() -  AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                            {
                                                AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_T_LEFT].CopyObject(tempUnknownCorner);
                                                tempUnknownCorner.setVisibility(false);
                                                recheck = true;
                                            }
                                            // yellow left post should be populated with unknown data here.
                                            AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                            AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);
                                        }
                                        else if( ( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) < 0
                                                &&(AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible() == false
                                                || AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen()))
                                        {
                                            //qDebug("\nTARGET ACQUIRED, YELLOW right goal T       ..u\n");
                                            if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() -  AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                            {
                                                AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_T_RIGHT].CopyObject(tempUnknownCorner);
                                                tempUnknownCorner.setVisibility(false);
                                                recheck = true;
                                            }
                                            // yellow left post should be populated with unknown data here.
                                            AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                            AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);
                                        }
                                    }
                                }
                                if(        (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() == true)
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                        && (tempUnknownCorner.isObjectVisible() == true)
                                        && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].ScreenX()- cornerPoints[x].PosX ) < POST_T_LIMIT )
                                        && (( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) > 0 )
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance() < 350)){

                                        //qDebug("\nTARGET ACQUIRED, YELLOW left goal T       ..\n");
                                    if(tempUnknownCorner.measuredDistance()!= 0 &&   fabs(tempUnknownCorner.measuredDistance() -  AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                    {
                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_T_LEFT].CopyObject(tempUnknownCorner);
                                        tempUnknownCorner.setVisibility(false);
                                    }
                                }
                                if( 	   (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible() == true)
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                        && (tempUnknownCorner.isObjectVisible() == true)
                                        && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) < POST_T_LIMIT )
                                        && (( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].ScreenX()- cornerPoints[x].PosX ) < 0 )
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance() < 350)){

                                        //qDebug("\nTARGET ACQUIRED, YELLOW right goal T       ..\n");
                                    if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() -  AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance()) < MAX_DISTANCE_T_GOAL)
                                    {
                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_T_RIGHT].CopyObject(tempUnknownCorner);
                                        tempUnknownCorner.setVisibility(false);
                                    }

                                }
                            }// end if((cornerPoints[x]->Orientation == 3)........
			} // end if (cornerPoints[x]->CornerType == 1)
			//-----------end Goal T and goal post combo rule ----------------------------------------------------------------------------------
		
		
			//--------------penalty L and goal post combo rule:--------------------------------------------------------------------------------
			//  included a half field range limit because of false positives on center circle
                        if (cornerPoints[x].CornerType == 0)
                        {
                            if( ((cornerPoints[x].Orientation == 4)||(cornerPoints[x].Orientation == 2))
                                && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() == true)
                                && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                && (tempUnknownCorner.isObjectVisible() == true)
                                && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].ScreenX()- cornerPoints[x].PosX ) < POST_L_LIMIT)
                                && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance() < 350)) {

                                //qDebug("\nTARGET ACQUIRED, YELLOW left penalty L      =\n");
                                    if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() -  AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                    {
                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_PEN_LEFT].CopyObject(tempUnknownCorner);
                                        tempUnknownCorner.setVisibility(false);
                                    }
                                }
		
                            if ( 	((cornerPoints[x].Orientation == 3)||(cornerPoints[x].Orientation == 2))
                                && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible() == true)
                                && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].TimeLastSeen() == tempUnknownCorner.TimeLastSeen())
                                && (tempUnknownCorner.isObjectVisible() == true)
                                && (fabs( AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].ScreenX() - cornerPoints[x].PosX ) < POST_L_LIMIT)
                                && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance() < 350)) {

                                //qDebug("\nTARGET ACQUIRED, YELLOW right penalty L      =\n");
                                if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() -  AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                {
                                    AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_PEN_RIGHT].CopyObject(tempUnknownCorner);
                                    tempUnknownCorner.setVisibility(false);
                                }
                            }
                            for(unsigned int FO_Counter = 0; FO_Counter < AllObjects->ambiguousFieldObjects.size(); FO_Counter++)
                            {

                                if(AllObjects->ambiguousFieldObjects[FO_Counter].getID() != FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN
                                   || AllObjects->ambiguousFieldObjects[FO_Counter].isObjectVisible() == false) continue;

                                if (    (cornerPoints[x].Orientation == 4)
                                    && (tempUnknownCorner.isObjectVisible() == true)
                                    && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() == false
                                            || AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen())
                                    && (fabs( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX()- cornerPoints[x].PosX ) < POST_L_LIMIT)
                                    && (( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) > 0)
                                    && (AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance() < 350)) {

                                    //qDebug("\nTARGET ACQUIRED, YELLOW left penalty L      u=\n");
                                    if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() -  AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                    {
                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_PEN_LEFT].CopyObject(tempUnknownCorner);
                                        tempUnknownCorner.setVisibility(false);
                                        recheck = true;
                                    }
                                    // yellow left post should be populated with unknown data here.
                                    AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                    AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);
                                }

                                else if (   (cornerPoints[x].Orientation == 3)
                                        && (tempUnknownCorner.isObjectVisible() == true)
                                        && (AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible() == false
                                            || AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].TimeLastSeen() != tempUnknownCorner.TimeLastSeen())
                                        && (fabs(AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX() - cornerPoints[x].PosX ) < POST_L_LIMIT)
                                        && (( AllObjects->ambiguousFieldObjects[FO_Counter].ScreenX()- cornerPoints[x].PosX ) < 0)
                                        && (AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance() < 350)) {

                                    //qDebug("\nTARGET ACQUIRED, YELLOW right penalty L      u=\n");
                                    if(tempUnknownCorner.measuredDistance()!= 0 &&  fabs(tempUnknownCorner.measuredDistance() -  AllObjects->ambiguousFieldObjects[FO_Counter].measuredDistance()) < MAX_DISTANCE_L_GOAL)
                                    {
                                        AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_PEN_RIGHT].CopyObject(tempUnknownCorner);
                                        tempUnknownCorner.setVisibility(false);
                                        recheck = true;
                                    }
                                    // yellow left post should be populated with unknown data here.
                                    AllObjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].CopyObject(AllObjects->ambiguousFieldObjects[FO_Counter]);
                                    AllObjects->ambiguousFieldObjects[FO_Counter].setVisibility(false);
                                }
                            }
                        }
                        if(tempUnknownCorner.isObjectVisible() == false)
                        {
                            CheckedCornerPoints[x] = true;
                        }

			//-------------- end penalty L and goal post combo rule ---------------------------------------------------------------------

		//////////////////////////////////////////////////// END YELLOW Goal ////////////////////////////////////////////////////////////////
		}
	}

        //Assigning all unchecked corners to Ambiguous Objects
        for (x = 0; x < cornerPoints.size(); x++){
            if(CheckedCornerPoints[x] == true) continue;
            TempID = 0;
            //qDebug("Checking CornerID: %i \t",x);
            //ASSIGNING T, L or X corner To TempID

            AmbiguousObject tempUnknownCorner;

            if (cornerPoints[x].CornerType == 0)
            {
                if(cornerPoints[x].Orientation == POINT_UP)
                {
                    TempID = FieldObjects::FO_CORNER_UNKNOWN_INSIDE_L;
                    tempUnknownCorner = AmbiguousObject(TempID, "Unknown Inside L");
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_FIELD_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_FIELD_RIGHT);

                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_RIGHT);

                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_RIGHT);

                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_FIELD_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_FIELD_RIGHT);

                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_RIGHT);

                }
                else
                {
                    TempID = FieldObjects::FO_CORNER_UNKNOWN_OUTSIDE_L;
                    tempUnknownCorner = AmbiguousObject(TempID, "Unknown Outside L");

                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_RIGHT);

                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_RIGHT);

                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_RIGHT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_LEFT);
                    tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_RIGHT);

                }

                //qDebug("FO_CORNER_UNKNOWN_L located \n");
            }
            else if (cornerPoints[x].CornerType == 1) //create new type for cross and check here. add definition in globals FO_CORNER_UNKNOWN_X
            {
                TempID = FieldObjects::FO_CORNER_UNKNOWN_T;
                tempUnknownCorner = AmbiguousObject(TempID, "Unknown T");
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_LEFT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_RIGHT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_LEFT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_RIGHT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_LEFT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_RIGHT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_LEFT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_RIGHT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_LEFT);
                tempUnknownCorner.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_RIGHT);
                //qDebug("FO_CORNER_UNKNOWN_T located \n");
            }
            else if (cornerPoints[x].CornerType > 1)
            {
                //qDebug("FO_CORNER_UNKNOWN_X located \n");
                continue;
            }

            //Initialising Variables
            GetDistanceToPoint(cornerPoints[x].PosX, cornerPoints[x].PosY, &TempDist, &TempBearing, &TempElev, vision);

            if(TempDist > 600 || TempDist == 0) continue; //Too Big distances, or no body angle information(0)

            Vector3<float> measured(TempDist,TempBearing,TempElev);
            Vector3<float> measuredError(0,0,0);
            Vector2<int> screenPosition(cornerPoints[x].PosX, cornerPoints[x].PosY);
            Vector2<float> screenPositionAngle(vision->CalculateBearing(screenPosition.x), vision->CalculateElevation(screenPosition.y));
            Vector2<int> sizeOnScreen(4,4);
            tempUnknownCorner.UpdateVisualObject(measured,measuredError,screenPositionAngle,screenPosition,sizeOnScreen,timestamp);
            AllObjects->ambiguousFieldObjects.push_back(tempUnknownCorner);

        }
    }


/**----------------------
// Method: 	DecodePenaltySpot
// Arguments: 	void
// Returns: 	Void
// Description: 4 scenarios of penalty spots using Goals/Posts and Centrecircle
------------------**/

void LineDetection::DecodePenaltySpot(FieldObjects* AllObjects, double timestamp)
{
    bool blueGoalSeen = false;
    bool yellowGoalSeen = false;
    bool centreCircleSeen = false;
    double yDist = 0.0;
    double bDist = 0.0;
    double cDist = 0.0;
    double pDist = 0.0;
    double goalBearing = 0.0;

    //Search for Posts or Unknown Posts:
    //qDebug() << "Looking for Amb Objects:";
    for(unsigned int i = 0; i < AllObjects->ambiguousFieldObjects.size(); i++)
    {
        if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
        {
                blueGoalSeen = true;
                bDist = AllObjects->ambiguousFieldObjects[i].measuredDistance();
                goalBearing =  AllObjects->ambiguousFieldObjects[i].measuredBearing();
        }
        else if(AllObjects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
        {
                yellowGoalSeen = true;
                yDist = AllObjects->ambiguousFieldObjects[i].measuredDistance();
                goalBearing =  AllObjects->ambiguousFieldObjects[i].measuredBearing();
        }
    }
    //qDebug() << "Looking for Known Objects:";
    for(int i = FieldObjects::FO_BLUE_LEFT_GOALPOST; i <= FieldObjects::FO_YELLOW_RIGHT_GOALPOST; i++)
    {
        if(AllObjects->stationaryFieldObjects[i].isObjectVisible())
        {
            if( i == FieldObjects::FO_BLUE_LEFT_GOALPOST || i == FieldObjects::FO_BLUE_RIGHT_GOALPOST)
            {
                    blueGoalSeen = true;
                    bDist = AllObjects->stationaryFieldObjects[i].measuredDistance();
                    goalBearing =  AllObjects->stationaryFieldObjects[i].measuredBearing();
            }
            else if ( i == FieldObjects::FO_YELLOW_LEFT_GOALPOST || i == FieldObjects::FO_YELLOW_RIGHT_GOALPOST)
            {
                    yellowGoalSeen = true;
                    yDist = AllObjects->stationaryFieldObjects[i].measuredDistance();
                    goalBearing =  AllObjects->stationaryFieldObjects[i].measuredBearing();
            }
        }
    }

    //qDebug() << "Decoding Penalty Spots: " << possiblePenaltySpots.size();
    for(unsigned int i = 0; i < possiblePenaltySpots.size(); i++)
    {
        if(possiblePenaltySpots[i].getID() == FieldObjects::FO_PENALTY_UNKNOWN)
        {
            pDist = possiblePenaltySpots[i].measuredDistance();
        }
        else
        {
            //qDebug() << "No Distance to Penalty";
            continue;
        }


        if(yellowGoalSeen && blueGoalSeen)
        {
                //qDebug() << "Cannot see both yellow and blue Goals at once." << std::endl;
                //AllObjects->ambiguousFieldObjects.push_back(possiblePenaltySpots[i]);
                return;
        }
        if(yellowGoalSeen==false && blueGoalSeen==false)
        {
                //qDebug() << "No Goals or Posts seen." << std::endl;
                //AllObjects->ambiguousFieldObjects.push_back(possiblePenaltySpots[i]);
                return;
        }
        if(AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].TimeLastSeen() == timestamp)
        {
                cDist = AllObjects->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].measuredDistance();
                centreCircleSeen = true;
        }
        //else
        //{
                //std::cout << "No CentreCircle Visible" << std::endl;
                //AllObjects->ambiguousFieldObjects.push_back(possiblePenaltySpots[i]);
                //return;
        //}

        //Scenarios: Spot is:
        //1. in between Blue Goals & CentreCircle -> BluePenaltySpot
        //2. in between Yellow Goals & CentreCircle -> YellowPenaltySpot
        //3. in front of Centre Circle & BlueGoals -> YellowPenaltySpot
        //4. in front of CentreCircle & YellowGoals -> BluePenaltySpot

        //Sort by vision Distance to work out the position of PenaltySpot
        /*if(bDist > pDist && cDist < pDist && blueGoalSeen && centreCircleSeen)
        {
            //BluePenaltySpot
            AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_BLUE].CopyObject(possiblePenaltySpots[i]);
            //fieldObjects[FO_PENALTY_UNKNOWN].seen = false;
            continue;
        }

        else if(yDist > pDist && cDist < pDist && yellowGoalSeen && centreCircleSeen)
        {
            //yellowPenaltySpot
             AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_YELLOW].CopyObject(possiblePenaltySpots[i]);
            //fieldObjects[FO_PENALTY_UNKNOWN].seen = false;
            continue;
        }

        else if(bDist > pDist && cDist > pDist && centreCircleSeen && blueGoalSeen)
        {
            //yellowPenaltySpot
             AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_YELLOW].CopyObject(possiblePenaltySpots[i]);
            //fieldObjects[FO_PENALTY_UNKNOWN].seen = false;
            continue;
        }
        else if(yDist > pDist && cDist > pDist && yellowGoalSeen && centreCircleSeen)
        {
            //bluePenaltySpot
             AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_BLUE].CopyObject(possiblePenaltySpots[i]);
            //fieldObjects[FO_PENALTY_UNKNOWN].seen = false;
            continue;
        }*/

        if(yellowGoalSeen)
        {
            //Calculate the transformed distance difference:
            float xpspot = possiblePenaltySpots[i].measuredDistance() * sin(possiblePenaltySpots[i].measuredBearing());
            float ypspot = possiblePenaltySpots[i].measuredDistance() * cos(possiblePenaltySpots[i].measuredBearing());
            float xgoal = yDist * sin(goalBearing);
            float ygoal = yDist * cos(goalBearing);

            float measuredTransformedDistance = sqrt((xpspot - xgoal)*(xpspot - xgoal) + (ypspot - ygoal)*(ypspot - ygoal));
            //yellowPenaltySpot
            //qDebug() << "Distance Between the Goal and Penalty spot: "<<measuredTransformedDistance;
            if(fabs(measuredTransformedDistance - 193.1321) < 50)
            {
                AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_YELLOW].CopyObject(possiblePenaltySpots[i]);
            }
            else if(fabs(measuredTransformedDistance - (600 - 193.1321)) < 50)
            {
                AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_BLUE].CopyObject(possiblePenaltySpots[i]);
            }
            //fieldObjects[FO_PENALTY_UNKNOWN].seen = false;
            continue;
        }

        else if(blueGoalSeen)
        {
            //Calculate the transformed distance difference:
            float xpspot = possiblePenaltySpots[i].measuredDistance() * sin(possiblePenaltySpots[i].measuredBearing());
            float ypspot = possiblePenaltySpots[i].measuredDistance() * cos(possiblePenaltySpots[i].measuredBearing());
            float xgoal = bDist * sin(goalBearing);
            float ygoal = bDist * cos(goalBearing);

            float measuredTransformedDistance = sqrt((xpspot - xgoal)*(xpspot - xgoal) + (ypspot - ygoal)*(ypspot - ygoal));
            //bluePenaltySpot
            //qDebug() << "Distance Between the Goal and Penalty spot: "<<measuredTransformedDistance;
            if(fabs(measuredTransformedDistance - 193.1321) < 50)
            {
                AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_BLUE].CopyObject(possiblePenaltySpots[i]);
            }
            else if(fabs(measuredTransformedDistance - (600 - 193.1321)) < 100)
            {
                AllObjects->stationaryFieldObjects[FieldObjects::FO_PENALTY_YELLOW].CopyObject(possiblePenaltySpots[i]);
            }
            continue;
        }
        else
        {
            //AllObjects->ambiguousFieldObjects.push_back(possiblePenaltySpots[i]);
        }
        return;
    }
}

bool LineDetection::GetDistanceToPoint(double cx, double cy, double* distance, double* bearing, double* elevation, Vision* vision)
{
    *bearing = vision->CalculateBearing(cx);
    *elevation = vision->CalculateElevation(cy);

    std::vector<float> ctgvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector); 
    if(isOK == true)
    {
        Matrix camera2groundTransform = Matrix4x4fromVector(ctgvector);
        Vector3<float> result;
        result = Kinematics::DistanceToPoint(camera2groundTransform, *bearing, *elevation);
        *distance = result[0];
        *bearing = result[1];
        *elevation = result[2];

        #if DEBUG_VISION_VERBOSITY > 6
            debug << "\t\tCalculated Distance to Point: " << *distance<<std::endl;
        #endif
    }
    return isOK;
}

bool LineDetection::GetDistanceToPoint(LinePoint point, Vector3<float> &relativePoint, Vision* vision)
{
    #if DEBUG_VISION_VERBOSITY > 5
        debug << "\t\t Calculated Bearing and Elevation: " << std::endl;
    #endif
    float bearing = vision->CalculateBearing(point.x);
    float elevation = vision->CalculateElevation(point.y);
    #if DEBUG_VISION_VERBOSITY > 5
        debug << "\t\t Bearing and Elevation: " << bearing << elevation << std::endl;
    #endif
    std::vector<float> ctgvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    #if DEBUG_VISION_VERBOSITY > 5
        debug << "\t\t Get Sensor Data:  " << isOK << std::endl;
    #endif
    if(isOK == true)
    {
        Matrix camera2groundTransform = Matrix4x4fromVector(ctgvector);

        relativePoint = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);

        #if DEBUG_VISION_VERBOSITY > 5
            debug << "\t\tCalculated Distance to Point: " << std::endl;
        #endif
    }
    return isOK;
}

bool LineDetection::GetDistanceToPoint(Point point, Vector3<float> &relativePoint, Vision* vision)
{
    float bearing = vision->CalculateBearing(point.x);
    float elevation = vision->CalculateElevation(point.y);

    std::vector<float> ctgvector;
    bool isOK = vision->getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    if(isOK == true)
    {
        Matrix camera2groundTransform = Matrix4x4fromVector(ctgvector);

        relativePoint = Kinematics::DistanceToPoint(camera2groundTransform, bearing, elevation);

        //#if DEBUG_VISION_VERBOSITY > 6
        //    debug << "\t\tCalculated Distance to Point: " << *distance<<std::endl;
        //#endif
    }
    return isOK;
}



/****
  Using the Bresenham's_line_algorithm to calculate which pixels to classify
****/

bool LineDetection::CheckGreenBetweenTwoPoints(int x0,int y0,int x1,int y1, Vision* vision)
{
    bool foundColour = false;
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    int temp;
    if(steep)
    {
        //swap(x0, y0);
        temp = x0;
        x0 = y0;
        y0 = temp;

        //swap(x1, y1)
        temp = x1;
        x1 = y1;
        y1 = temp;
    }
    if (x0 > x1)
    {
        //swap(x0, x1);
        temp = x0;
        x0 = x1;
        x1 = temp;

        //swap(y0, y1);
        temp = y0;
        y0 = y1;
        y1 = temp;
    }
    int deltax = x1 - x0;

    int deltay = abs(y1 - y0);

    float error = 0;
    float deltaerr = (float)deltay / (float)deltax;
    int ystep;
    int y = y0;
    if(y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for (int x = x0; x < x1; x++)
    {
        if (steep)
        {
            //qDebug() << y << x;
            //Classify(y,x);
            if(vision->classifyPixel(y,x) == ClassIndex::green)
            {
                //qDebug() << y << x;
                foundColour = true;
                break;
            }
        }
        else
        {
            //Classify(x,y);
            //qDebug() << x << y;
            if(vision->classifyPixel(x,y) == ClassIndex::green)
            {

                foundColour = true;
                break;
            }
        }
        error = error + deltaerr;
        //qDebug() << error << deltaerr;
        if(error >= 0.5)
        {
             y = y + ystep;
             error = error - 1.0;

        }
    }
    return foundColour;

}

void LineDetection::MergeCloseCorners()
{
    //Pixel Distance before Merging:
    int minDistance = 10;
    std::vector < CornerPoint > ::iterator cornerPointIterator1;
    std::vector < CornerPoint > ::iterator cornerPointIterator2;
    for(cornerPointIterator1 = cornerPoints.begin(); cornerPointIterator1 < cornerPoints.end(); )
    {

        for(cornerPointIterator2 = cornerPointIterator1+1; cornerPointIterator2  < cornerPoints.end(); )
        {
            float d1 = DistanceBetweenTwoPoints(cornerPointIterator1->PosX,cornerPointIterator1->PosY,cornerPointIterator2->PosX,cornerPointIterator2->PosY);
            if(d1 < minDistance && cornerPointIterator1->CornerType == cornerPointIterator2->CornerType)
            {

                cornerPointIterator1->PosX = (cornerPointIterator2->PosX+cornerPointIterator1->PosX)/2;
                cornerPointIterator1->PosY = (cornerPointIterator2->PosY+cornerPointIterator1->PosY)/2;

                //Remove i and j
                cornerPointIterator2 = cornerPoints.erase(cornerPointIterator2);
                cornerPointIterator1 = cornerPoints.begin();
                cornerPointIterator2 = cornerPointIterator1 + 1;

            }
            else
            {
                ++cornerPointIterator2;
            }

        }
        ++cornerPointIterator1;
    }

}

float LineDetection::DistanceBetweenTwoPoints(float x0, float y0, float x1, float y1)
{
    return (float)sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}


/*
void LineDetection::GetDistanceToPoint(double cx, double cy, double* distance, double* bearing, double* elevation) {
 
  //robot_console_printf("GDTP2 called\n");
// assumptions:   cameratilt = 0 is horizontal
//		  cameratilt > 0 is 'down'
// 		  body tilt  > 0 leaning forward
// body roll not taken into consideration at this time


//For the horizontal field of view, Section 7.4 of the hardware documentation claims the FOV is '58 degrees diagonal'. My interpretation of this is that it would be corner to corner.
IF the screen resolution is 640x480, this means from centre to corner is 29degrees, which is sqrt(320^2+240^2)=400 pixels. Then the focal length will be 400/tan(29) = 721.6pixels.
(This translates backwards to a horizontal field of view of arctan(320/721.6) = 23.91 degrees.) (0.417308 rads)

    //double horizontal_FOV = FOVxSIM;//0.83459;  //160x120  //reworked
    double horizontal_FOV = FOVx;
    double CAMERA_TILT_OFFSET = - 0.035;   //ADD 40 DEGREES TO THIS WHEN USING BOTTOM CAMERA // this is a guesss at possible camera offset of 2 degrees  // define this at top

    Kinematics *kin = &Kinematics::getInstance();
    double CameraTilt = CAMERA_TILT_OFFSET + kin->m_headPitch;
    double CameraPan = kin->m_headYaw;
//    double BodyTilt = currentBodyTilt;
    double BodyTilt = kin->m_bodyPitch;

    //float bodyRoll = currentBodyRoll;   // not taken into consideration at this time

    double focalLength;
    double Ximage = cx;
    double Yimage = cy;

    double Vcam[3];
    double V1[3];
    double V2[3];
    double V3[3];

    // Camera tilt matrix
    double CTM[3][3] =     { {cos(CameraTilt),   0,    sin(CameraTilt)},
                           {      0, 	    	 1,         0	      }, 
			   {-sin(CameraTilt),    0,    cos(CameraTilt)}   };   

    // camera pan matrix
    double CPM[3][3] =     { {cos(CameraPan),   -sin(CameraPan),  0},		
                           { sin(CameraPan),    cos(CameraPan),  0}, 
			   {      0,  		     0,          1}   };       

    // body tilt matrix
    double BTM[3][3] =     {{cos(BodyTilt),   0,    sin(BodyTilt)},		
                           {      0, 	      1,         0	 }, 
			   {-sin(BodyTilt),   0,    cos(BodyTilt)}   };    	

    double alpha;
    double height;
    //double line[3];

    focalLength =  (IMAGE_WIDTH/2) / (tan(horizontal_FOV/2)) ;	//horizontal

    Vcam[0] = focalLength;
    Vcam[1] = (IMAGE_WIDTH/2) - Ximage;
    Vcam[2] = (IMAGE_HEIGHT/2) - Yimage;
	
    //V1[0] = cos(CameraTilt)*Vcam[0]+sin(CameraTilt)*Vcam[2];

    V1[0] = CTM[0][0]*Vcam[0] + CTM[0][1]*Vcam[1] + CTM[0][2]*Vcam[2];
    V1[1] = CTM[1][0]*Vcam[0] + CTM[1][1]*Vcam[1] + CTM[1][2]*Vcam[2];
    V1[2] = CTM[2][0]*Vcam[0] + CTM[2][1]*Vcam[1] + CTM[2][2]*Vcam[2];

    V2[0] = CPM[0][0]*V1[0] + CPM[0][1]*V1[1] + CPM[0][2]*V1[2];
    V2[1] = CPM[1][0]*V1[0] + CPM[1][1]*V1[1] + CPM[1][2]*V1[2];
    V2[2] = CPM[2][0]*V1[0] + CPM[2][1]*V1[1] + CPM[2][2]*V1[2];

    V3[0] = BTM[0][0]*V2[0] + BTM[0][1]*V2[1] + BTM[0][2]*V2[2];
    V3[1] = BTM[1][0]*V2[0] + BTM[1][1]*V2[1] + BTM[1][2]*V2[2];
    V3[2] = BTM[2][0]*V2[0] + BTM[2][1]*V2[1] + BTM[2][2]*V2[2];

    height = 51.5;  
    alpha = - height/V3[2];

    *distance =  alpha *( sqrt( V3[0]*V3[0] + V3[1]*V3[1] )   );
    *bearing =  atan2 ( V3[1] , V3[0] );
    *elevation = - atan2 ( height, *distance );  // positive elevation is up
}
*/



void LineDetection::swap(std::vector<LinePoint> &array, int i, int j)
{
        LinePoint temp;
        //qDebug() << "Swapping "<< i << "," <<j;
        temp     = array[i];
        array[i] = array[j];
        array[j] = temp;
}
/*
0 UNDEFINED
1 Y - Smallest to Largest Y
2 X - Smallest to Largest X
*/
void LineDetection::qsort(std::vector<LinePoint> &array, int left, int right, int type)
{

        int current;
        int last;

        if( left >= right )
                return;

        if (right - left < 5) {
        // Do a selection sort  this doesnt seem to make it much faster ;(
        int c;
        for(int j = left; j < right; j++) {
        c = j;
        for (int k = j + 1; k < right+1; k++) {
                if(type==1)
                {
                if(array[k].y < array[j].y) {
                c = k;
                if (c!=j) swap( array, c, j );
                }
                }
                else if(type==2)
                {
                if(array[k].x < array[j].x) {
                c = k;
                if (c!=j) swap( array, c, j );
                }
                }

        }
        }
        return;
        }

        swap( array, left, (left+right)/2);
        last = left;


        for( current = left+1; current <= right; ++current )
        {
                if(type==1)
                {
                    if(array[current].y < array[left].y) {
                            ++last;
                            swap( array, last, current);
                    }
                }
                else if(type==2)
                {
                    if(array[current].x < array[left].x) {
                            ++last;
                            swap( array, last, current);
                    }
                }
        }

        swap( array, left, last );

        qsort( array, left,   last-1, type );
        qsort( array, last+1, right,type );
}


void LineDetection::swap(std::vector<LSFittedLine> &array, int i, int j)
{
        LSFittedLine temp;

        temp     = array[i];
        array[i] = array[j];
        array[j] = temp;
}

void LineDetection::qsort(std::vector<LSFittedLine> &array, int left, int right)
{

        int current;
        int last;

        if( left >= right )
                return;

        if (right - left < 5) {
        // Do a selection sort  this doesnt seem to make it much faster ;(
        int c;
        for(int j = left; j < right; j++) {
                c = j;
                for (int k = j + 1; k < right+1; k++) {
                        if(array[k].leftPoint.x < array[j].leftPoint.x && array[j].valid && array[k].valid) {
                                c = k;
                                if (c!=j) swap( array, c, j);
                        }
                }
        }
        return;
        }

        swap( array, left, (left+right)/2);
        last = left;


        for( current = left+1; current <= right; ++current )
        {
                if(array[current].leftPoint.x < array[left].leftPoint.x && array[current].valid && array[left].valid) {
                        ++last;
                        swap( array, last, current);
                }

        }

        swap( array, left, last);

        qsort( array, left,   last-1);
        qsort( array, last+1, right);
}
