#ifndef SAM_H_SHANNON
#define SAM_H_SHANNON

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include "../robocup/Tools/Math/LSFittedLine.h"
#include "../robocup/Tools/Math/Matrix.h"
#include "../robocup/Tools/Math/Vector3.h"

//maximum field objects rules
#define MAX_POINTS 500
#define MAX_LINES 15
//splitting rules
#define SPLIT_DISTANCE 2
#define MIN_POINTS_OVER 2
//Noise splitting rules
#define SPLIT_NOISE_ITERATIONS 2
//merging rules
#define MAX_GRAD_DIFF 1
#define MAX_INTERCEPT_DIFF 10
#define MAX_RHO_DIFF 12
#define MAX_PHI_DIFF 0.5
#define MAX_END_POINT_DIFF 5
//Line keeping rules
#define MIN_POINTS_TO_LINE 3
#define MIN_POINTS_TO_LINE_FINAL 5
#define MIN_LINE_R2_FIT 0.9997
//Debug rules
#define DEBUG 0
#define DEBUG_POINTS 0
#define DEBUG_SPLIT 0
#define DEBUG_SEPARATE 0
#define DEBUG_MERGE 0
#define DEBUG_SPLIT_NOISE 0
#define DEBUG_SHOULD_SPLIT 0
#define DEBUG_CLEAR_SMALL 0

using namespace std;

class SAM
{
public:

    static vector<LinePoint*> noisePoints;
    static unsigned int noFieldLines;

    //GENERIC
    static void initDebug(ofstream& dout);

    //LEAST-SQUARES FITTING
    static void splitAndMergeLS(vector<LSFittedLine*>& lines, vector<LinePoint*>& points, bool clearsmall=true, bool cleardirty=true, bool noise=true);
    //CLUSTERS
    static void splitAndMergeLSClusters(vector<LSFittedLine*>& lines, vector< vector<LinePoint*> >& clusters, vector<LinePoint*> leftover, bool clearsmall=true, bool cleardirty=true, bool noise=true);
    //POINTCONVERSION
    static void convertPoints(vector<LinePoint*>& points, Vector3<double> vals);

private:
    //DEBUGGING
    /*
    static ofstream* debug_out;

    static void debugPrint(const string& s);
    static void debugPrint(const Line& l);
    static void debugPrint(const LinePoint& p);
    static void debugPrint(const vector<LSFittedLine*>& lines);
    static void debugPrint(const vector<LinePoint*>& points);
    */

    //LEAST-SQUARES FITTING
    static void splitLS(vector<LSFittedLine*>& lines, vector<LinePoint*>& points);
    static void splitLSIterative(vector<LSFittedLine*>& lines, vector<LinePoint*>& points);
    static void splitNoiseLS(vector<LSFittedLine*>& lines);
    static void mergeLS(vector<LSFittedLine*>& lines);
    static void generateLSLine(LSFittedLine& line, vector<LinePoint*>& points);
    static void sortLinesLS(vector<LSFittedLine*>& lines);
    static bool separateLS(vector<LinePoint*>& left, vector<LinePoint*>& right, LinePoint* split_point, LSFittedLine& line);


    //GENERIC
    static void findFurthestPoint(LSFittedLine& line, unsigned int& points_over, unsigned int& furthest_point);
    static void addToNoise(LinePoint* point);
    static void clearSmallLines(vector<LSFittedLine*>& lines);
    static void clearDirtyLines(vector<LSFittedLine*>& lines);
    static bool shouldMergeLines(const LSFittedLine& line1, const LSFittedLine& line2);

};

#endif // SAM_H_SHANNON
