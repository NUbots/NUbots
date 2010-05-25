#ifndef LINEDETECTION_H_DEFINED
#define LINEDETECTION_H_DEFINED

#include "../Tools/Math/LSFittedLine.h"
#include "CornerPoint.h"
#include "ClassifiedSection.h"
//#include "../Globals.h"
//#include "../FieldObject.h"
//#include "../Kinematics/Horizon.h"
//#include "CircleFitting.h"
#include <iostream>
#include "FieldObjects/FieldObjects.h"
class Vision;
class NUSensorsData;
class Kinematics;

#define MAX_LINEPOINTS 100
#define MAX_FIELDLINES 15
#define MAX_CORNERPOINTS 10
#define VERT_POINT_THICKNESS 40
#define MIN_POINT_THICKNESS 2
#define HORZ_POINT_THICKNESS 40

#define POST_T_LIMIT (320) // set as define at top later  ALSO  these values need to be a defined fraction of the IMAGE_WIDTH
#define POST_L_LIMIT (320) // more thought needs to be given to these limits

#define POINT_UP 1
#define POINT_DOWN 2
#define POINT_RIGHT 3
#define POINT_LEFT 4
#define OUT_BOUNDS_VIEW_RIGHT 1
#define OUT_BOUNDS_VIEW_LEFT 2
#define POINT_UP_AND_LEFT 3
#define POINT_UP_AND_RIGHT 4




class LineDetection{

	public:
	//VARIABLES:
        std::vector<LinePoint> linePoints;
        std::vector<LSFittedLine> fieldLines;
        std::vector<CornerPoint> cornerPoints;
        std::vector<AmbiguousObject> possiblePenaltySpots;
        //int LinePointCounter;
        //int FieldLinesCounter;
        //int CornerPointCounter;


	//METHODS:
	LineDetection();
    	~LineDetection();
        void FormLines(ClassifiedSection* scanArea, int image_width, int image_height, int spacing, FieldObjects* AllObjects, Vision* vision, NUSensorsData* data);
	
	
	private:

        int TotalValidLines;
        int LINE_SEARCH_GRID_SIZE;
        int PenaltySpotLineNumber;
        NUSensorsData* sensorsData;
        Kinematics* kin;

        void FindLinePoints(ClassifiedSection* scanArea,Vision* vision,int image_width, int image_height);
        void FindFieldLines(int image_width,int image_height);

        bool DetectWhitePixels(int checkX, int checkY, int searchRadius,Vision* vision);
        void FindPenaltySpot(Vision* vision);
        void DecodePenaltySpot(FieldObjects* AllObjects, float timestamp);

        void FindCornerPoints(int image_width,int image_height);
        void DecodeCorners(FieldObjects* AllObjects, float timestamp,  Vision* vision);
        void GetDistanceToPoint(double,double,double*,double*,double*, Vision* vision);

        //! Line Point Sorting
        void qsort(std::vector<LinePoint> &array, int left, int right, int type);
        void swap(std::vector<LinePoint> &array, int i, int j);

        //! Lines Sorting
        void qsort(std::vector<LSFittedLine> &array, int left, int right);
        void swap(std::vector<LSFittedLine> &array, int i, int j);
}
;

#endif

