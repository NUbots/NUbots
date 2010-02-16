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

//#define MAX_LINEPOINTS 1000
//#define MAX_FIELDLINES 200
//#define MAX_CORNERPOINTS 50
#define VERT_POINT_THICKNESS 36
#define MIN_POINT_THICKNESS 1
#define HORZ_POINT_THICKNESS 36

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

        //int LinePointCounter;
        //int FieldLinesCounter;
        //int CornerPointCounter;


	//METHODS:
	LineDetection();
    	~LineDetection();
        void FormLines(ClassifiedSection* scanArea,int image_width, int image_height, int spacing);
	
	
	private:
        int IMAGE_WIDTH;
	int TotalValidLines;
        int LINE_SEARCH_GRID_SIZE;
        void FindLinePoints(ClassifiedSection* scanArea);
        void FindFieldLines(int image_width,int image_height);
        void FindCornerPoints(int image_height);
	void DecodeCorners();
	void GetDistanceToPoint(double,double,double*,double*,double*);
        void qsort(std::vector<LinePoint> array, int left, int right, int type);
        void swap(std::vector<LinePoint> array, int i, int j);

}
;

#endif

