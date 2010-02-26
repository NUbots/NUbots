#include "LineDetection.h"
#include "TransitionSegment.h"
#include "ClassificationColours.h"
#include <math.h>
#define MIN_POINTS_ON_LINE_FINAL 5
#define MIN_POINTS_ON_LINE 4
//#define LINE_SEARCH_GRID_SIZE 4
//#define POINT_SEARCH_GRID_SIZE 4
#include <stdio.h>
//using namespace std;
#include <vector>
//#include "../Kinematics/Kinematics.h"
//#include <QDebug>

LineDetection::LineDetection(){

        //LinePointCounter = 0;
        //FieldLinesCounter = 0;
        //CornerPointCounter = 0;
	TotalValidLines = 0;
}

LineDetection::~LineDetection(){
return;
}

void LineDetection::FormLines(ClassifiedSection* scanArea,int image_width, int image_height, int spacing) {
	//RESETING VARIABLES
        LINE_SEARCH_GRID_SIZE = spacing;
        for(unsigned int i =0; i < linePoints.size() ; i++ )
	{
		linePoints[i].x =0;
		linePoints[i].y =0;
	}

        for(unsigned int i =0; i < fieldLines.size() ; i++ )
	{
		fieldLines[i].clearPoints();
	}
        //LinePointCounter = 0;
        TotalValidLines = 0;
        //FieldLinesCounter = 0;
        //CornerPointCounter = 0;
	//FIND LINE POINTS:
        FindLinePoints(scanArea);
        FindFieldLines(image_width,image_height);
        FindCornerPoints(image_height);
        //DecodeCorners();
}

void LineDetection::FindLinePoints(ClassifiedSection* scanArea)
{
    int numberOfLines = scanArea->getNumberOfScanLines();
    int maxLengthOfScanLine = 0;

    //Find Length of Longest ScanLines as we want to only have longest scan lines.
    for(int i = 0; i < numberOfLines; i++)
    {
        if(scanArea->getScanLine(i)->getLength() > maxLengthOfScanLine)
        {
                maxLengthOfScanLine = scanArea->getScanLine(i)->getLength();
        }
    }
    for(int i = 0; i< numberOfLines; i++)
    {
        int numberOfSegments = scanArea->getScanLine(i)->getNumberOfSegments();
        for(int j = 0; j < numberOfSegments; j++)
        {
            if(scanArea->getScanLine(i)->getLength() < maxLengthOfScanLine/1.5) continue;
            TransitionSegment* segment = scanArea->getScanLine(i)->getSegment(j);
            //int segmentSize = segment->getSize();
            //CHECK SIZE
            if(segment->getSize() < MIN_POINT_THICKNESS || segment->getSize() > VERT_POINT_THICKNESS ) continue;

            //CHECK COLOUR(GREEN-WHITE-GREEN Transistion)
            //CHECK COLOUR (U-W-G or G-W-U Transistion)
            if(     ClassIndex::green   ==  segment->getBeforeColour()
                &&  ClassIndex::white   ==  segment->getColour()
                &&  ClassIndex::green   ==  segment->getAfterColour())
                    /*(ClassIndex::white   ==  segment->getColour() && ClassIndex::green   ==  segment->getAfterColour())
                    || (ClassIndex::white   ==  segment->getColour() && ClassIndex::green   ==  segment->getBeforeColour()))*/

            {
                //ADD A FIELD LINEPOINT!
                Vector2<int>start = segment->getStartPoint();
                Vector2<int>end = segment->getEndPoint();

                LinePoint tempLinePoint;
                tempLinePoint.width = (int)segment->getSize();
                tempLinePoint.x = round((start.x + end.x) / 2);
                tempLinePoint.y = round((start.y + end.y) / 2);

                //CUT CLOSE LINE POINTS OFF
                bool canNotAdd = false;
                for (unsigned int num =0; num <linePoints.size(); num++)
                {
                    if((fabs(tempLinePoint.x - linePoints[num].x) < 1.0))
                    {
                        if((fabs(tempLinePoint.y - linePoints[num].y) < 1.0))
                        {
                            canNotAdd = true;
                        }
                    }
                }
                if(!canNotAdd)
                {
                    tempLinePoint.inUse = false;
                    linePoints.push_back(tempLinePoint);
                }
                //// qDebug() << "Found LinePoint (MidPoint): "<< (start.x + end.x) / 2 << ","<< (start.y+end.y)/2 << " Length: "<< segment->getSize();
                //LinePointCounter++;

            }
        }

    }
    // qDebug() << "Number of LinePoints: " << linePoints.size();
    return;
}

void LineDetection::FindFieldLines(int IMAGE_WIDTH, int IMAGE_HEIGHT){

    //printf("Start Find Lines..\n");

    //Try and make lines now...
    int DistanceStep;
    double ColSlopeVal;
    int previousPointID;

    //Lines 'should' have a point on the same search pattern that maches them, which should also be close to them..
    //First look for horz lines (from the vert search grid..)
    // ***********************************************************************************************************************
    if ( linePoints.size() < MIN_POINTS_ON_LINE_FINAL)
    {
        return;
    }
    //SORT THE LINES BY Y then BY X:

    //qsort(linePoints,0,linePoints.size()-1,2);


    //Only bother searching if there is enough points to make part of a line..
    for (unsigned int SearchFrom = 0; SearchFrom < linePoints.size()-1; SearchFrom++)
    {   //for all line points recorded

        if(linePoints[SearchFrom].inUse) continue;
        if(linePoints[SearchFrom].width > VERT_POINT_THICKNESS) continue;  //STOP if LINE is too THICK, but can use if in Vertical Line Search.
        for (unsigned int EndCheck = SearchFrom+1; EndCheck < linePoints.size()-1; EndCheck++){ 	//for remaining points recorded
            if(linePoints[EndCheck].width > VERT_POINT_THICKNESS) continue; //STOP if LINE is too THICK, but can use if in Vertical Line Search.
            if (!(linePoints[EndCheck].inUse == false) && (linePoints[EndCheck].x != linePoints[SearchFrom].x))continue;

                // Skip all points on the same search line as this one or have already been removed..
            if (linePoints[EndCheck].x <= linePoints[SearchFrom].x+LINE_SEARCH_GRID_SIZE*2)
            {
                DistanceStep = (int)(linePoints[EndCheck].x-linePoints[SearchFrom].x)/LINE_SEARCH_GRID_SIZE;  //number of grid units long
                //// qDebug() << linePoints[SearchFrom].y << ","<<linePoints[EndCheck].y << LINE_SEARCH_GRID_SIZE*DistanceStep << fabs(linePoints[SearchFrom].y - linePoints[EndCheck].y);
                if (fabs(linePoints[SearchFrom].y - linePoints[EndCheck].y) <= LINE_SEARCH_GRID_SIZE*2*DistanceStep)//fabs(LINE_SEARCH_GRID_SIZE*DistanceStep))
                   // && fabs(linePoints[SearchFrom].y - linePoints[EndCheck].y) < LINE_SEARCH_GRID_SIZE)
                {
                    //We've found what might be a line, so lets see if we can find any more lines that match this one..
                    LSFittedLine tempFieldLine;
                    tempFieldLine.addPoint(linePoints[SearchFrom]);
                    tempFieldLine.addPoint(linePoints[EndCheck]);
                    previousPointID = EndCheck;
                    ColSlopeVal = linePoints[SearchFrom].y - linePoints[EndCheck].y;
                    //loop through the rest of the points that maybe in this 'line'
                    for (unsigned int PointID = EndCheck+1; PointID < linePoints.size(); PointID++){
                        if (linePoints[previousPointID].x == linePoints[PointID].x) continue;
                        if (fabs(linePoints[PointID].x - linePoints[previousPointID].x) < int(LINE_SEARCH_GRID_SIZE*2)){
                            double DisMod = (linePoints[PointID].x - linePoints[previousPointID].x)/LINE_SEARCH_GRID_SIZE;
                            //Check if the slope is about right..
                            if (fabs(linePoints[PointID].y+(ColSlopeVal*DisMod) - linePoints[previousPointID].y) <= 1){
                                //This is another point on the line..
                                tempFieldLine.addPoint(linePoints[PointID]);
                                previousPointID = PointID;
                            }
                        }
                        else
                        {
                            PointID = linePoints.size();
                        }
                    }
                    fieldLines.push_back(tempFieldLine);
                }

                if(fieldLines.size()==0) continue;
                else if (fieldLines.back().numPoints < MIN_POINTS_ON_LINE)
                {
                    fieldLines.back().clearPoints();
                    fieldLines.pop_back();
                }
            }
            //else
            //{
            //    EndCheck = linePoints.size();
            //}

        }
    }

    //Now do all that again, but this time looking for the vert lines from the horz search grid..

    //SORT POINTS
    //// qDebug() << "SORTING...";
    //qsort(linePoints,0,linePoints.size()-1,2);
    //qDebug() << "Finnished...";
    for (unsigned int SearchFrom = 0; SearchFrom < linePoints.size()-1; SearchFrom++){
        if(linePoints[SearchFrom].inUse) continue;
        if(linePoints[SearchFrom].width > VERT_POINT_THICKNESS) continue;  //STOP if LINE is too THICK, but can use if in Vertical Line Search.
        if(linePoints[SearchFrom].width < MIN_POINT_THICKNESS*3) continue;
        for (unsigned int EndCheck = SearchFrom+1; EndCheck < linePoints.size(); EndCheck++){
                //std::cout << "Comparing.."<< SearchFrom << " with " << EndCheck <<std::endl;
                //Skip all points on the same search line as this one or have already been removed..
            if (!((linePoints[EndCheck].inUse == false) && (linePoints[EndCheck].y != linePoints[SearchFrom].y)))continue;
            if(linePoints[EndCheck].width > VERT_POINT_THICKNESS) continue;  //STOP if LINE is too THICK, but can use if in Vertical Line Search.
            if(linePoints[EndCheck].width < MIN_POINT_THICKNESS*3) continue;
            if (linePoints[EndCheck].y <= linePoints[SearchFrom].y+LINE_SEARCH_GRID_SIZE*4)
            {
                DistanceStep = (int)(linePoints[EndCheck].y-linePoints[SearchFrom].y)/LINE_SEARCH_GRID_SIZE;
                if (fabs(linePoints[SearchFrom].x - linePoints[EndCheck].x) < int(LINE_SEARCH_GRID_SIZE*4*DistanceStep))
                   // && fabs(linePoints[SearchFrom].x - linePoints[EndCheck].x) < LINE_SEARCH_GRID_SIZE)
                    {
                //We've found what might be a line, so lets see if we can find any more lines that match this one..
                    //std::cout << "Starting New vertical Line" << std::endl;
                    LSFittedLine tempFieldLine;
                    tempFieldLine.addPoint(linePoints[SearchFrom]);
                    tempFieldLine.addPoint(linePoints[EndCheck]);
                    previousPointID = EndCheck;
                    ColSlopeVal = linePoints[SearchFrom].x - linePoints[EndCheck].x;

                    for (unsigned int PointID = EndCheck+1; PointID < linePoints.size(); PointID++){
                        if (linePoints[previousPointID].y == linePoints[PointID].y) continue;
                        if (fabs(linePoints[previousPointID].y - linePoints[PointID].y) < LINE_SEARCH_GRID_SIZE*4){
                            double DisMod = (linePoints[PointID].y - linePoints[previousPointID].y)/LINE_SEARCH_GRID_SIZE;
                            //Check if the slope is about right..
                            if (fabs(linePoints[PointID].x+(ColSlopeVal*DisMod) - linePoints[previousPointID].x) <= 1){
                                    //This is another point on the line..
                                    tempFieldLine.addPoint(linePoints[PointID]);
                                    previousPointID = PointID;
                            }
                        }
                        else {
                                //We've moved too far to keep searching..
                                PointID = linePoints.size();
                        }
                    }
                    fieldLines.push_back(tempFieldLine);
                }
                if (fieldLines.size()==0)continue;
                else if(fieldLines.back().numPoints < MIN_POINTS_ON_LINE)
                {
                    //This is a bad line, so throw it out..
                    fieldLines.back().clearPoints();
                    fieldLines.pop_back();
                }
            }
            //else {
            //        EndCheck = linePoints.size();
            //}
        }
    }



    //---------------------------------------------------
    // START OF JOINING LINES
    //---------------------------------------------------

    unsigned int LineIDStart;

    //Drop out here if there isn't any point continuing.. DEBUG:
    if (fieldLines.size() < 1)
    {
            printf("No Lines... ABORT!!!!");
            // qDebug() << "No Lines... ABORT!!!!";
            return;
    }
    else{
            printf("END FieldLinesCounter: %i\n", fieldLines.size());
            // qDebug() << "Before Joining FieldLinesCounter: " << fieldLines.size();
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

    //We should now have about 10 lines max, some of which can be joined together (since they may be two lines seperated by a break but otherwise one line really...)
    for (LineIDStart = 0; LineIDStart < fieldLines.size()-1; LineIDStart++){
            if (!fieldLines[LineIDStart].valid) continue;   	// this apears to me to be first use of 'validLine' so how does it get to be true? ALEX
            for (unsigned int LineIDEnd = LineIDStart+1; LineIDEnd<fieldLines.size(); LineIDEnd++){
                    if (!fieldLines[LineIDEnd].valid) continue;
                    //Try extending the lines so they are near the ends of the other ones, and see if their in any way close...

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
                    r2tls2 = Line2.getr2tls();


                    //Now make sure the slopes are both about the same degree angle....
                    // Seems to have a problem with lines "within" other lines, so pick them out..
                    // qDebug() << "Joining Line " <<LineIDStart <<"-"<<LineIDEnd <<": " <<r2tls1 << "," <<r2tls2 << ", "<<MSD1 << ", "<<MSD2;
                    if ((r2tls1 > .99 && r2tls2 > .99 && MSD1 < 20  && MSD2 < 20))// || (r2tls1 > .90 && r2tls2 > .90 && MSD2 < 20 && fabs(Line1.getGradient()) > 1))                    // (.90 & 40)alex CAN ADJUST THIS FOR LINE JOINING
                    {
                        //They are the same line, so join them together...
                        // qDebug() << "Joining Lines: "<< LineIDEnd<< " to "<<LineIDStart;
                        fieldLines[LineIDStart].joinLine(fieldLines[LineIDEnd]);
                        //std::cout << "Num Points Line2: "<< fieldLines[LineIDEnd].numPoints <<std::endl;
                        fieldLines[LineIDEnd].clearPoints();

                    }
            }
    }

    //Remove any lines that are still too small (or really badly fitted)..
    for (LineIDStart = 0; LineIDStart < fieldLines.size(); LineIDStart++){
        if (!fieldLines[LineIDStart].valid) continue;
        if (fieldLines[LineIDStart].numPoints < MIN_POINTS_ON_LINE_FINAL  || fieldLines[LineIDStart].getr2tls() < .80){// alex ADJUST CAN HELP TO DELETE CIRCLE LINES
            fieldLines[LineIDStart].valid = false;

            //std::cout<<"Line " << LineIDStart << " was Removed." << std::endl;
        }
        else {
            TotalValidLines++;
            //printf( "VaildLine Found: %i @ %i\n" ,TotalValidLines, LineIDStart);
        }

    }
    // qDebug() << "Total number of Vaild Lines: "<<TotalValidLines;


    for (unsigned int i = 0; i < fieldLines.size(); i++)
    {
        // qDebug() << i<< ": \t Valid: "<<fieldLines[i].valid
        //        << " \t Start(x,y): ("<< fieldLines[i].leftPoint.x<<","<< fieldLines[i].leftPoint.y
        //        << ") \t EndPoint(x,y):(" << fieldLines[i].rightPoint.x<<","<< fieldLines[i].rightPoint.y<< ")"
        //        << "\t Number of LinePoints: "<< fieldLines[i].numPoints;
    }

}


/*------------------
// Method: 	FindCornerPoints
// Arguments: 	void
// Returns: 	Void
// Description: Finds cornerpoints from lines that intersect
//		Stores corner points in global variable "cornerPoints".
------------------*/

void LineDetection::FindCornerPoints(int IMAGE_HEIGHT){

	//std::cout<<"Starting to Look for Corners.."<< std::endl;
	if (TotalValidLines <2)
	{
		//printf("%i Valid Lines... ABORT\n",TotalValidLines);
		return;
	}
	//else
		//printf("TotalValidLines: %i\n",TotalValidLines);
  
	int CommonX, CommonY;
	unsigned short Top,Bottom,Left,Right,Type;
  
  //Now try and find where the lines intersect.
        for (unsigned int LineIDStart = 0; LineIDStart < fieldLines.size()-1; LineIDStart++){
		if (!fieldLines[LineIDStart].valid) continue;
		//See if this line intersects with any other ones...
                for (unsigned int LineIDCheck = LineIDStart+1; LineIDCheck < fieldLines.size(); LineIDCheck++){
			if (!fieldLines[LineIDCheck].valid) continue;
			//Check this lines' slops are far enough apart...
                        if (!(fabs(fieldLines[LineIDStart].getAngle() - fieldLines[LineIDCheck].getAngle()) >=.08)) continue;
			//std::cout << "Comparing Line Angles: " << LineIDStart << ", " << LineIDCheck<< std::endl;
			//this seems to be very oblique?? make more acute for circle stuff ALEX
			//Work out their intersecting X pos..
			CommonX = 	(int)((fieldLines[LineIDStart].getYIntercept() - fieldLines[LineIDCheck].getYIntercept()) / 
					(fieldLines[LineIDCheck].getGradient() - fieldLines[LineIDStart].getGradient()));
			//Check if they intersect on the screen.. (or near enough..)
			if (!CommonX > 0 && CommonX < IMAGE_WIDTH) continue;
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
                                                tempCornerPoint.Orientation = POINT_UP; // was -1 is 1
					else
                                                tempCornerPoint.Orientation = POINT_DOWN;  // was 1 is 2
				}
				else {
					if (Left >= Right)
                                                tempCornerPoint.Orientation = POINT_RIGHT;  //4
					else 
                                                tempCornerPoint.Orientation = POINT_LEFT;  //3
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
                        cornerPoints.push_back(tempCornerPoint);

		}
	}
        //qDebug() << "Total Corners Found: " << cornerPoints.size();
        for (unsigned int i = 0; i < cornerPoints.size() ; i++)
        {
            // qDebug() << i << ": \t "<< cornerPoints[i].PosX << ","<< cornerPoints[i].PosY;
        }
}


/*------------------
// Method: 	DecodeCorners
// Arguments: 	void
// Returns: 	Void
// Description: Assigns known corner poitns to field objects
------------------*/

/*

void LineDetection::DecodeCorners(){
	
	double TempDist, TempBearing, TempElev;
	int TempID;
	int x;
	bool recheck = false;

        if (CornerPointCounter>4)                  // ********  this filters out center circle. only a count 0f 2 is checked.
	{
		//PERFORM ELIPSE FIT HERE!
		return;  // identify cross here?  ALEX
	}

	//CHECK EACH CORNER POINT:	
	for (x = 0; x < CornerPointCounter; x++){
		TempID = 0;
		//printf("Checking CornerID: %i \n",x);
		//ASSIGNING T, L or X corner To TempID
		if (cornerPoints[x].CornerType == 0)
			TempID = FO_CORNER_UNKNOWN_L;
		else if (cornerPoints[x].CornerType == 1) //create new type for cross and check here. add definition in globals FO_CORNER_UNKNOWN_X
     	     		TempID = FO_CORNER_UNKNOWN_T;
		else if (cornerPoints[x].CornerType > 1)
			//printf("FO_CORNER_UNKNOWN_X located \n");
		
		//START DECODING T
		if (TempID){
			//Initialising Variables
			GetDistanceToPoint(cornerPoints[x].PosX, cornerPoints[x].PosY, &TempDist, &TempBearing, &TempElev);
			fieldObjects[TempID].seen = true;
			fieldObjects[TempID].visionBearing = TempBearing;
			fieldObjects[TempID].visionDistance = TempDist;
			fieldObjects[TempID].visionElevation = TempElev;
			fieldObjects[TempID].visionX = cornerPoints[x].PosX;
			fieldObjects[TempID].visionY = cornerPoints[x].PosY;

			//--------------Goal T and goal post combo rule:----------------------------------------------------------------------------------
			if (cornerPoints[x].CornerType == 1){  // must include a half field range limit because of false positives on center circle
				if((cornerPoints[x].Orientation == 3) || (cornerPoints[x].Orientation == 4)){ // could divide limit by distance to get a range adjustment		
					if( 	(fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].seen == true) 
						&& ( (ABS((fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionX) - (cornerPoints[x].PosX))) < POST_T_LIMIT )  
						&& (fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionDistance < 350)){

						if( ( fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) > 0 ){ 
							//printf("\nTARGET ACQUIRED, BLUEleft goal T       ..u\n");
							recheck = true;
							//COPY: fieldObjects[TempID] TO fieldObjects[FO_CORNER_BLUE_T_LEFT]
							fieldObjects[FO_CORNER_BLUE_T_LEFT].CopyVisionData(fieldObjects[TempID]);
							fieldObjects[TempID].seen = false;
							//COPY: fieldObjects[FO_BLUE_GOALPOST_UNKNOWN] TO fieldObjects[FO_BLUE_LEFT_GOALPOST]
							fieldObjects[FO_BLUE_LEFT_GOALPOST].CopyVisionData(fieldObjects[FO_BLUE_GOALPOST_UNKNOWN]);
							fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].seen = false;
						}
						else if( ( fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < 0 ){ 
							//printf("\nTARGET ACQUIRED, BLUE right goal T       ..u\n");
							recheck = true;
							////COPY: fieldObjects[TempID] TO fieldObjects[FO_CORNER_BLUE_T_RIGHT]
							fieldObjects[FO_CORNER_BLUE_T_RIGHT].CopyVisionData(fieldObjects[TempID]);
							fieldObjects[TempID].seen = false;
							//COPY: fieldObjects[FO_BLUE_GOALPOST_UNKNOWN] TO fieldObjects[FO_BLUE_RIGHT_GOALPOST]
							fieldObjects[FO_BLUE_RIGHT_GOALPOST].CopyVisionData(fieldObjects[FO_BLUE_GOALPOST_UNKNOWN]);
							fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].seen = false;
						}
					}
					if( 	(fieldObjects[FO_BLUE_LEFT_GOALPOST].seen == true) 
						&& (fieldObjects[TempID].seen == true)
						&& (ABS( fieldObjects[FO_BLUE_LEFT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_T_LIMIT )
						&& (( fieldObjects[FO_BLUE_LEFT_GOALPOST].visionX - cornerPoints[x].PosX ) > 0 ) 
						&& (fieldObjects[FO_BLUE_LEFT_GOALPOST].visionDistance < 350)){

						//printf("\nTARGET ACQUIRED, BLUE left goal T      ..\n");
						//COPY: fieldObjects[TempID] TO fieldObjects[FO_CORNER_BLUE_T_LEFT]
						fieldObjects[FO_CORNER_BLUE_T_LEFT].CopyVisionData(fieldObjects[TempID]);
						fieldObjects[TempID].seen = false;
					}
					if( 	(fieldObjects[FO_BLUE_RIGHT_GOALPOST].seen == true)
						&& (fieldObjects[TempID].seen == true)
						&& (ABS( fieldObjects[FO_BLUE_RIGHT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_T_LIMIT )
						&& (( fieldObjects[FO_BLUE_RIGHT_GOALPOST].visionX - cornerPoints[x].PosX ) < 0 )
						&& (fieldObjects[FO_BLUE_RIGHT_GOALPOST].visionDistance < 350)){

						//printf("\nTARGET ACQUIRED,  BLUE right goal T       ..\n");
						//GetDistanceToPoint2(cornerPoints[x]->PosX, cornerPoints[x]->PosY, &TempDist, &TempBearing, &TempElev);
						//GetDistanceToPoint(cornerPoints[x]->PosX, cornerPoints[x]->PosY, &TempDist, &TempBearing, &TempElev);
						fieldObjects[FO_CORNER_BLUE_T_RIGHT].CopyVisionData(fieldObjects[TempID]);
						fieldObjects[TempID].seen = false;
					}
				}// end if((cornerPoints[x]->Orientation == 3)........
			} // end if (cornerPoints[x]->CornerType == 1)
			//-----------end Goal T and goal post combo rule ----------------------------------------------------------------------------------


			//--------------penalty L and goal post combo rule:--------------------------------------------------------------------------------
			// must include a half field range limit because of false positives on center circle
			if ( 	(cornerPoints[x].CornerType == 0)
				&& ((cornerPoints[x].Orientation == 4)||(cornerPoints[x].Orientation == 2))
				&& (fieldObjects[FO_BLUE_LEFT_GOALPOST].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_BLUE_LEFT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (fieldObjects[FO_BLUE_LEFT_GOALPOST].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, BLUE left penalty L       =\n");
				fieldObjects[FO_CORNER_BLUE_PEN_LEFT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
			}
		
			if ( 	(cornerPoints[x].CornerType == 0)
				&& ((cornerPoints[x].Orientation == 3)||(cornerPoints[x].Orientation == 2))
				&& (fieldObjects[FO_BLUE_RIGHT_GOALPOST].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_BLUE_RIGHT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (fieldObjects[FO_BLUE_RIGHT_GOALPOST].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, BLUE right penalty L       =\n");
				fieldObjects[FO_CORNER_BLUE_PEN_RIGHT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
			}
		
			if ( 	(cornerPoints[x].CornerType == 0)
				&& (cornerPoints[x].Orientation == 4)
				&& (fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (( fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) > 0)
				&& (fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, BLUE left penalty L       u=\n");
				recheck = true;
				fieldObjects[FO_CORNER_BLUE_PEN_LEFT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
				// blue left post should be populated with unknown data here.
				fieldObjects[FO_BLUE_LEFT_GOALPOST].CopyVisionData(fieldObjects[FO_BLUE_GOALPOST_UNKNOWN]);
				fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].seen = false;
			}
		
			if ( 	(cornerPoints[x].CornerType == 0)
				&& (cornerPoints[x].Orientation == 3)
				&& (fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (( fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < 0)
				&& (fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, BLUE right penalty L      u=\n");
				recheck = true;
				fieldObjects[FO_CORNER_BLUE_PEN_RIGHT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
				
				// blue right post should be populated with unknown data here.
				fieldObjects[FO_BLUE_RIGHT_GOALPOST].CopyVisionData(fieldObjects[FO_BLUE_GOALPOST_UNKNOWN]);
				fieldObjects[FO_BLUE_GOALPOST_UNKNOWN].seen = false;
			}
			//-------------- end penalty L and goal post combo rule ---------------------------------------------------------------------------
			//////////////////////////////////////////////////// END BLUE Goal ////////////////////////////////////////////////////////////////

			//--------------Goal T and goal post combo rule:----------------------------------------------------------------------------------
			if (cornerPoints[x].CornerType == 1){  // should include a half field range limit because of false positives on center circle
			
				if((cornerPoints[x].Orientation == 3) || (cornerPoints[x].Orientation == 4)){// could divide limit by distance to get a range adjustment		
			 
					if( 	(fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].seen == true) 
						&& (fieldObjects[TempID].seen == true)
						&& ( (ABS((fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionX) - (cornerPoints[x].PosX))) < POST_T_LIMIT )  
						&& (fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionDistance < 350)){
						
						if( ( fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) > 0 ){ 
							//printf("\nTARGET ACQUIRED, YELLOW left goal T       ..u\n");
							fieldObjects[FO_CORNER_YELLOW_T_LEFT].CopyVisionData(fieldObjects[TempID]);
							fieldObjects[TempID].seen = false;
							recheck = true;
							// yellow left post should be populated with unknown data here.
							fieldObjects[FO_YELLOW_LEFT_GOALPOST].CopyVisionData(fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN]);
							fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].seen = false;
						}
						else if( ( fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < 0 ){ 
							//printf("\nTARGET ACQUIRED, YELLOW right goal T       ..u\n");
							fieldObjects[FO_CORNER_YELLOW_T_RIGHT].CopyVisionData(fieldObjects[TempID]);
							fieldObjects[TempID].seen = false;
							recheck = true;
							// yellow left post should be populated with unknown data here.
							fieldObjects[FO_YELLOW_RIGHT_GOALPOST].CopyVisionData(fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN]);
							fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].seen = false;
						}
					}
					if(        (fieldObjects[FO_YELLOW_LEFT_GOALPOST].seen == true) 
						&& (fieldObjects[TempID].seen == true)
						&& (ABS( fieldObjects[FO_YELLOW_LEFT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_T_LIMIT )
						&& (( fieldObjects[FO_YELLOW_LEFT_GOALPOST].visionX - cornerPoints[x].PosX ) > 0 ) 
						&& (fieldObjects[FO_YELLOW_LEFT_GOALPOST].visionDistance < 350)){
						
						//printf("\nTARGET ACQUIRED, YELLOW left goal T       ..\n");

						fieldObjects[FO_CORNER_YELLOW_T_LEFT].CopyVisionData(fieldObjects[TempID]);
						fieldObjects[TempID].seen = false;
					}
					if( 	   (fieldObjects[FO_YELLOW_RIGHT_GOALPOST].seen == true) 
						&& (fieldObjects[TempID].seen == true)
						&& (ABS( fieldObjects[FO_YELLOW_RIGHT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_T_LIMIT )
						&& (( fieldObjects[FO_YELLOW_RIGHT_GOALPOST].visionX - cornerPoints[x].PosX ) < 0 ) 
						&& (fieldObjects[FO_YELLOW_RIGHT_GOALPOST].visionDistance < 350)){		  
						
						//printf("\nTARGET ACQUIRED, YELLOW right goal T       ..\n");
						fieldObjects[FO_CORNER_YELLOW_T_RIGHT].CopyVisionData(fieldObjects[TempID]);
						fieldObjects[TempID].seen = false;
					
					}
				}// end if((cornerPoints[x]->Orientation == 3)........
			} // end if (cornerPoints[x]->CornerType == 1)
			//-----------end Goal T and goal post combo rule ----------------------------------------------------------------------------------
		
		
			//--------------penalty L and goal post combo rule:--------------------------------------------------------------------------------
			//  included a half field range limit because of false positives on center circle
			if ( 	(cornerPoints[x].CornerType == 0)
				&& ((cornerPoints[x].Orientation == 4)||(cornerPoints[x].Orientation == 2))
				&& (fieldObjects[FO_YELLOW_LEFT_GOALPOST].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_YELLOW_LEFT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (fieldObjects[FO_YELLOW_LEFT_GOALPOST].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, YELLOW left penalty L      =\n");
				fieldObjects[FO_CORNER_YELLOW_PEN_LEFT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
					
			}
		
			if ( 	(cornerPoints[x].CornerType == 0)
				&& ((cornerPoints[x].Orientation == 3)||(cornerPoints[x].Orientation == 2))
				&& (fieldObjects[FO_YELLOW_RIGHT_GOALPOST].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_YELLOW_RIGHT_GOALPOST].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (fieldObjects[FO_YELLOW_RIGHT_GOALPOST].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, YELLOW right penalty L      =\n");
				fieldObjects[FO_CORNER_YELLOW_PEN_RIGHT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
			}
		
			if ( 	(cornerPoints[x].CornerType == 0)
				&& (cornerPoints[x].Orientation == 4)
				&& (fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (( fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) > 0)
				&& (fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, YELLOW left penalty L      u=\n");
				fieldObjects[FO_CORNER_YELLOW_PEN_LEFT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
				recheck = true;
				// yellow left post should be populated with unknown data here.
				fieldObjects[FO_YELLOW_LEFT_GOALPOST].CopyVisionData(fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN]);
				fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].seen = false;
			}
		
			if ( 	(cornerPoints[x].CornerType == 0)
				&& (cornerPoints[x].Orientation == 3)
				&& (fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].seen == true)
				&& (fieldObjects[TempID].seen == true)
				&& (ABS( fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < POST_L_LIMIT)  
				&& (( fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionX - cornerPoints[x].PosX ) < 0)
				&& (fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].visionDistance < 350)) {

				//printf("\nTARGET ACQUIRED, YELLOW right penalty L      u=\n");
				fieldObjects[FO_CORNER_YELLOW_PEN_RIGHT].CopyVisionData(fieldObjects[TempID]);
				fieldObjects[TempID].seen = false;
				recheck = true;
				// yellow left post should be populated with unknown data here.
				fieldObjects[FO_YELLOW_RIGHT_GOALPOST].CopyVisionData(fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN]);
				fieldObjects[FO_YELLOW_GOALPOST_UNKNOWN].seen = false;
			}
			
			if(recheck ==true)
			{
				recheck =false;
				x = -1;
				//printf("Resetting X to perform Recheck.... \n");
			}
			//-------------- end penalty L and goal post combo rule ---------------------------------------------------------------------

		//////////////////////////////////////////////////// END YELLOW Goal ////////////////////////////////////////////////////////////////
		}
	}
}

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



void LineDetection::swap(std::vector<LinePoint> array, int i, int j)
{
        LinePoint temp;
        //// qDebug() << "Swapping "<< i << "," <<j;
        temp     = array[i];
        array[i] = array[j];
        array[j] = temp;
}
/*
0 UNDEFINED
1 Y - Smallest to Largest Y
2 X - Smallest to Largest X
*/
void LineDetection::qsort(std::vector<LinePoint> array, int left, int right, int type)
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
